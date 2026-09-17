#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <ostream>
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
			stopping_ = true;
		}
		cv_.notify_one();
		worker_.join();
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
			queue_.push_back(std::move(pending_));
			pending_.clear();
			cv_.notify_one();
		}
		return 0;
	}

private:
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
				lock.unlock();
				// The actually-slow work (writing to a console/pipe, flushing a log file)
				// happens here, off the caller's thread.
				target_->sputn(chunk.data(), static_cast<std::streamsize>(chunk.size()));
				target_->pubsync();
				lock.lock();
			}
			if (stopping_)
			{
				break;
			}
		}
	}

	std::ostream &stream_;
	std::streambuf *target_;
	std::mutex mutex_;
	std::condition_variable cv_;
	std::string pending_;
	std::deque<std::string> queue_;
	// stopping_ must be declared (and therefore constructed) before worker_: members
	// initialize in declaration order, and worker_'s constructor starts the background
	// thread immediately, which reads stopping_ from run() right away.
	bool stopping_ = false;
	std::thread worker_;
};
