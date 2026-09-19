#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
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
/// without bound, and stop() gives the drain thread a bounded window before abandoning it
/// rather than hanging indefinitely.
///
/// An abandoned drain thread keeps using the wrapped streambuf, so the caller must keep that
/// alive (or end the process) when stop() returns false — see finish_logging() in main.cpp.
class AsyncStreambuf : public std::streambuf
{
public:
	// Installs itself as stream's streambuf (like logging.cpp's TeeStreambuf), forwarding
	// everything to whatever streambuf stream had beforehand — the console, or a TeeStreambuf
	// if --log2file already wrapped it. Construct this AFTER any such wrapping so it ends up
	// as the outermost layer and can defer both console and file I/O equally.
	explicit AsyncStreambuf(std::ostream &stream) :
	  stream_(stream), target_(stream.rdbuf()), state_(std::make_shared<State>())
	{
		state_->target = target_;
		worker_ = std::thread(&AsyncStreambuf::run, state_);
		stream_.rdbuf(this);
	}

	~AsyncStreambuf() override
	{
		stop(std::chrono::seconds(2));
		stream_.rdbuf(target_); // Restore, mirroring TeeStreambuf's destructor.
	}

	/// @brief Hands everything still buffered to the drain thread and waits up to timeout for it
	/// to write it out. Safe to call more than once.
	/// @return true once the drain thread has finished; false if the sink was still stuck when
	/// the timeout ran out, in which case the thread is abandoned and keeps using the wrapped
	/// streambuf.
	bool stop(std::chrono::milliseconds timeout)
	{
		if (stopped_)
		{
			return !abandoned_;
		}
		stopped_ = true;

		{
			std::lock_guard<std::mutex> lock(state_->mutex);
			// Whatever's still buffered but never got an explicit flush (e.g. a trailing
			// '\n' with no std::endl/std::flush after it) must still reach the worker —
			// otherwise the last, possibly most important, line written before shutdown
			// is silently lost.
			if (!pending_.empty())
			{
				enqueue_locked(std::move(pending_));
				pending_.clear();
			}
			state_->stopping = true;
		}
		state_->cv.notify_one();

		// The target may be permanently stuck (a closed pipe nobody reads, a hung disk), in
		// which case the worker never returns from target->sputn()/pubsync() and a plain
		// join() would hang program shutdown forever on a dead sink. Give it a bounded
		// window to finish draining; if it's still not done, detach instead of hanging.
		// The worker only touches the shared State, which it keeps alive itself.
		const auto deadline = std::chrono::steady_clock::now() + timeout;
		while (!state_->finished.load() && std::chrono::steady_clock::now() < deadline)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (state_->finished.load())
		{
			worker_.join();
			return true;
		}
		worker_.detach();
		abandoned_ = true;
		return false;
	}

	AsyncStreambuf(const AsyncStreambuf &) = delete;
	AsyncStreambuf &operator=(const AsyncStreambuf &) = delete;

protected:
	int overflow(int ch) override
	{
		if (traits_type::eof() != ch)
		{
			std::lock_guard<std::mutex> lock(state_->mutex);
			pending_.push_back(static_cast<char>(ch));
		}
		return ch;
	}

	std::streamsize xsputn(const char *text, std::streamsize count) override
	{
		std::lock_guard<std::mutex> lock(state_->mutex);
		pending_.append(text, static_cast<std::size_t>(count));
		return count;
	}

	int sync() override
	{
		std::lock_guard<std::mutex> lock(state_->mutex);
		if (!pending_.empty())
		{
			enqueue_locked(std::move(pending_));
			pending_.clear();
			state_->cv.notify_one();
		}
		return 0;
	}

private:
	// Everything the drain thread touches lives here, owned jointly by the thread and this
	// object, so an abandoned thread never reads freed memory after this object is gone.
	struct State
	{
		std::mutex mutex;
		std::condition_variable cv;
		std::deque<std::string> queue;
		std::size_t queuedBytes = 0;
		std::size_t droppedBytes = 0;
		bool stopping = false;
		std::atomic<bool> finished{ false };
		std::streambuf *target = nullptr;
	};

	// Bounds how much can pile up if the sink stalls forever rather than just briefly — the
	// exact failure mode this wrapper exists to tolerate. Past this, the oldest queued output
	// is dropped (and the drop noted once it's safe to write again) instead of growing memory
	// without limit.
	static constexpr std::size_t kMaxQueuedBytes = 8 * 1024 * 1024;

	void enqueue_locked(std::string chunk)
	{
		state_->queuedBytes += chunk.size();
		state_->queue.push_back(std::move(chunk));
		while ((state_->queuedBytes > kMaxQueuedBytes) && (state_->queue.size() > 1))
		{
			state_->droppedBytes += state_->queue.front().size();
			state_->queuedBytes -= state_->queue.front().size();
			state_->queue.pop_front();
		}
	}

	static void run(std::shared_ptr<State> state)
	{
		std::unique_lock<std::mutex> lock(state->mutex);
		while (true)
		{
			state->cv.wait(lock, [&state] { return state->stopping || !state->queue.empty(); });
			while (!state->queue.empty())
			{
				std::string chunk = std::move(state->queue.front());
				state->queue.pop_front();
				state->queuedBytes -= chunk.size();
				std::size_t dropped = state->droppedBytes;
				state->droppedBytes = 0;
				lock.unlock();
				// The actually-slow work (writing to a console/pipe, flushing a log file)
				// happens here, off the caller's thread.
				if (0 != dropped)
				{
					std::ostringstream notice;
					notice << "[AsyncStreambuf] dropped " << dropped << " bytes of backlogged log output (sink couldn't keep up)\n";
					std::string noticeText = notice.str();
					state->target->sputn(noticeText.data(), static_cast<std::streamsize>(noticeText.size()));
				}
				state->target->sputn(chunk.data(), static_cast<std::streamsize>(chunk.size()));
				state->target->pubsync();
				lock.lock();
			}
			if (state->stopping)
			{
				break;
			}
		}
		state->finished.store(true);
	}

	std::ostream &stream_;
	std::streambuf *target_;
	std::shared_ptr<State> state_;
	std::string pending_; // Guarded by state_->mutex
	std::thread worker_;
	bool stopped_ = false;
	bool abandoned_ = false;
};
