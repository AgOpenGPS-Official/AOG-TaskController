#include "async_log.hpp"

#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <streambuf>
#include <string>
#include <thread>
#include <utility>

namespace
{
	// Past this many queued bytes the oldest lines are dropped, so a sink that never recovers can't
	// grow memory without limit.
	constexpr std::size_t maxQueuedBytes = 8 * 1024 * 1024;

	struct Queue
	{
		std::mutex mutex;
		std::condition_variable hasLines;
		std::condition_variable drained;
		std::deque<std::string> lines;
		std::size_t bytes = 0;
		std::size_t droppedBytes = 0;
		bool writing = false;
	};

	void write_lines(Queue &queue)
	{
		std::unique_lock<std::mutex> lock(queue.mutex);
		while (true)
		{
			queue.hasLines.wait(lock, [&queue] { return !queue.lines.empty(); });
			std::deque<std::string> batch;
			batch.swap(queue.lines);
			queue.bytes = 0;
			const std::size_t dropped = std::exchange(queue.droppedBytes, 0);
			queue.writing = true;
			lock.unlock();

			if (0 != dropped)
			{
				std::cout << "[AsyncLog] dropped " << dropped << " bytes of backlogged log output\n";
			}
			for (const std::string &line : batch)
			{
				std::cout << line;
			}
			std::cout << std::flush;

			lock.lock();
			queue.writing = false;
			if (queue.lines.empty())
			{
				queue.drained.notify_all();
			}
		}
	}

	// Never destroyed: the writer thread can still be blocked in std::cout when the process exits.
	Queue &queue()
	{
		static Queue *instance = [] {
			auto *created = new Queue;
			std::thread(write_lines, std::ref(*created)).detach();
			return created;
		}();
		return *instance;
	}

	void submit(std::string line)
	{
		Queue &q = queue();
		{
			std::lock_guard<std::mutex> lock(q.mutex);
			q.bytes += line.size();
			q.lines.push_back(std::move(line));
			while ((q.bytes > maxQueuedBytes) && (q.lines.size() > 1))
			{
				q.droppedBytes += q.lines.front().size();
				q.bytes -= q.lines.front().size();
				q.lines.pop_front();
			}
		}
		q.hasLines.notify_one();
	}

	// Collects one thread's output and hands it over a whole line at a time, so lines from different
	// threads never interleave and no stream state is shared between threads.
	class LineBuffer : public std::streambuf
	{
	public:
		~LineBuffer() override
		{
			if (!line.empty())
			{
				submit(std::move(line));
			}
		}

	protected:
		int overflow(int ch) override
		{
			if (traits_type::eof() != ch)
			{
				const char character = static_cast<char>(ch);
				append(&character, 1);
			}
			return ch;
		}

		std::streamsize xsputn(const char *text, std::streamsize count) override
		{
			append(text, static_cast<std::size_t>(count));
			return count;
		}

	private:
		void append(const char *text, std::size_t count)
		{
			line.append(text, count);
			const std::size_t end = line.rfind('\n');
			if (std::string::npos != end)
			{
				submit(line.substr(0, end + 1));
				line.erase(0, end + 1);
			}
		}

		std::string line;
	};
} // namespace

namespace async_log
{
	std::ostream &stream()
	{
		thread_local LineBuffer buffer;
		thread_local std::ostream out(&buffer);
		return out;
	}

	bool flush(std::chrono::milliseconds timeout)
	{
		Queue &q = queue();
		const auto deadline = std::chrono::steady_clock::now() + timeout;

		// try_lock rather than lock: this also runs from crash handlers, where the crashed thread may
		// have died holding the mutex.
		std::unique_lock<std::mutex> lock(q.mutex, std::defer_lock);
		while (!lock.try_lock())
		{
			if (std::chrono::steady_clock::now() >= deadline)
			{
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return q.drained.wait_until(lock, deadline, [&q] { return q.lines.empty() && !q.writing; });
	}
} // namespace async_log
