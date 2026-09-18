#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <ostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>

/// @brief Wraps an existing std::streambuf (the console, or logging.cpp's TeeStreambuf to a log
/// file) so a flush on the wrapped stream never blocks the calling thread on that buffer's own
/// I/O. Every std::endl calls flush() -> pubsync() -> this class's sync(); instead of writing (and
/// potentially blocking on a slow console pipe or a disk flush) right there, sync() just hands the
/// buffered bytes to a background thread and returns immediately.
///
/// Without this, a single log line that blocks — a full stdout pipe nobody is draining fast
/// enough, a slow disk fsync when --log2file is active — stalls whatever thread called it. In
/// AOG-TaskController that thread is also the one driving the ISOBUS transmit path (see
/// Application::update() in app.cpp), so a logging stall shows up as a transmit stall: periodic
/// sends (TECU speed, TC status) fall behind or stop while nothing about the CAN link itself is
/// wrong.
///
/// Two things this can't fix, by nature of the trade: the crash handler's std::cout fallback
/// (crash_handler.cpp) writes to std::cerr instead specifically to stay outside this wrapper,
/// since abnormal termination skips static destructors and would otherwise lose whatever was
/// still queued. And if the underlying sink never recovers at all (not just slow — genuinely
/// stuck forever), queued output is capped and the oldest of it is dropped rather than growing
/// without bound, and shutdown gives the drain thread a bounded window before detaching it rather
/// than hanging indefinitely.
class AsyncStreambuf : public std::streambuf
{
public:
	// Installs itself as stream's streambuf (like logging.cpp's TeeStreambuf), forwarding
	// everything to whatever streambuf stream had beforehand — the console, or a TeeStreambuf
	// if --log2file already wrapped it. Construct this AFTER any such wrapping so it ends up
	// as the outermost layer and can defer both console and file I/O equally.
	explicit AsyncStreambuf(std::ostream &stream) :
	  stream_(stream), target_(stream.rdbuf()), worker_(&AsyncStreambuf::run, this)
	{
		stream_.rdbuf(this);
	}

	~AsyncStreambuf() override
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			// Whatever's still buffered but never got an explicit flush (e.g. a trailing
			// '\n' with no std::endl/std::flush after it) must still reach the worker —
			// otherwise the last, possibly most important, line written before shutdown
			// is silently lost.
			if (!pending_.empty())
			{
				enqueue_locked(std::move(pending_));
				pending_.clear();
			}
			stopping_ = true;
		}
		cv_.notify_one();

		// target_ may be permanently stuck (a closed pipe nobody reads, a hung disk), in
		// which case the worker never returns from target_->sputn()/pubsync() and a plain
		// join() would hang program shutdown forever on a dead sink. Give it a bounded
		// window to finish draining; if it's still not done, detach instead of hanging —
		// the process is exiting either way, and the OS reclaims the thread.
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (!finished_.load() && std::chrono::steady_clock::now() < deadline)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (finished_.load())
		{
			worker_.join();
		}
		else
		{
			worker_.detach();
		}

		stream_.rdbuf(target_); // Restore, mirroring TeeStreambuf's destructor.
	}

	AsyncStreambuf(const AsyncStreambuf &) = delete;
	AsyncStreambuf &operator=(const AsyncStreambuf &) = delete;

protected:
	int overflow(int ch) override
	{
		if (traits_type::eof() != ch)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			pending_.push_back(static_cast<char>(ch));
		}
		return ch;
	}

	std::streamsize xsputn(const char *text, std::streamsize count) override
	{
		std::lock_guard<std::mutex> lock(mutex_);
		pending_.append(text, static_cast<std::size_t>(count));
		return count;
	}

	int sync() override
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!pending_.empty())
		{
			enqueue_locked(std::move(pending_));
			pending_.clear();
			cv_.notify_one();
		}
		return 0;
	}

private:
	// Bounds how much can pile up if the sink stalls forever rather than just briefly — the
	// exact failure mode this wrapper exists to tolerate. Past this, the oldest queued output
	// is dropped (and the drop noted once it's safe to write again) instead of growing memory
	// without limit.
	static constexpr std::size_t kMaxQueuedBytes = 8 * 1024 * 1024;

	void enqueue_locked(std::string chunk)
	{
		queuedBytes_ += chunk.size();
		queue_.push_back(std::move(chunk));
		while ((queuedBytes_ > kMaxQueuedBytes) && (queue_.size() > 1))
		{
			droppedBytes_ += queue_.front().size();
			queuedBytes_ -= queue_.front().size();
			queue_.pop_front();
		}
	}

	void run()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		while (true)
		{
			cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
			while (!queue_.empty())
			{
				std::string chunk = std::move(queue_.front());
				queue_.pop_front();
				queuedBytes_ -= chunk.size();
				std::size_t dropped = droppedBytes_;
				droppedBytes_ = 0;
				lock.unlock();
				// The actually-slow work (writing to a console/pipe, flushing a log file)
				// happens here, off the caller's thread.
				if (0 != dropped)
				{
					std::ostringstream notice;
					notice << "[AsyncStreambuf] dropped " << dropped << " bytes of backlogged log output (sink couldn't keep up)\n";
					std::string noticeText = notice.str();
					target_->sputn(noticeText.data(), static_cast<std::streamsize>(noticeText.size()));
				}
				target_->sputn(chunk.data(), static_cast<std::streamsize>(chunk.size()));
				target_->pubsync();
				lock.lock();
			}
			if (stopping_)
			{
				break;
			}
		}
		finished_.store(true);
	}

	std::ostream &stream_;
	std::streambuf *target_;
	std::mutex mutex_;
	std::condition_variable cv_;
	std::string pending_;
	std::deque<std::string> queue_;
	std::size_t queuedBytes_ = 0;
	std::size_t droppedBytes_ = 0;
	// stopping_ and finished_ must be declared (and therefore constructed) before worker_:
	// members initialize in declaration order, and worker_'s constructor starts the
	// background thread immediately, which reads stopping_ from run() right away.
	bool stopping_ = false;
	std::atomic<bool> finished_{ false };
	std::thread worker_;
};
