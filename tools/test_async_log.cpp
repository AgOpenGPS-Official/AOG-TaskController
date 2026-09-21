/**
 * @brief Unit tests for async_log, which keeps a slow console or log file off the calling thread.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "async_log.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

// Stands in for std::cout's buffer; blocks every write until released, like a pipe nobody reads.
class BlockingSink : public std::stringbuf
{
public:
	std::atomic<bool> release{ false };

protected:
	std::streamsize xsputn(const char *text, std::streamsize count) override
	{
		while (!release.load())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return std::stringbuf::xsputn(text, count);
	}
};

static void test_lines_reach_cout_in_order()
{
	std::printf("test_lines_reach_cout_in_order\n");

	std::stringbuf sink;
	auto *original = std::cout.rdbuf(&sink);
	async_log::stream() << "first " << 1 << std::endl;
	async_log::stream() << "second\n";
	const bool drained = async_log::flush(std::chrono::seconds(2));
	std::cout.rdbuf(original);

	check(drained, "flush() reports everything was written");
	check(sink.str() == "first 1\nsecond\n", "lines arrive complete and in order");
}

static void test_partial_line_waits_for_newline()
{
	std::printf("test_partial_line_waits_for_newline\n");

	std::stringbuf sink;
	auto *original = std::cout.rdbuf(&sink);
	async_log::stream() << "partial";
	async_log::flush(std::chrono::seconds(2));
	check(sink.str().empty(), "text without a newline is not written yet");
	async_log::stream() << " done" << std::endl;
	async_log::flush(std::chrono::seconds(2));
	std::cout.rdbuf(original);

	check(sink.str() == "partial done\n", "the line is written once it ends");
}

static void test_threads_do_not_interleave_lines()
{
	std::printf("test_threads_do_not_interleave_lines\n");

	std::stringbuf sink;
	auto *original = std::cout.rdbuf(&sink);
	std::vector<std::thread> threads;
	for (int id = 0; id < 4; ++id)
	{
		threads.emplace_back([id] {
			for (int i = 0; i < 200; ++i)
			{
				async_log::stream() << "thread" << id << " line " << i << " end" << std::endl;
			}
		});
	}
	for (auto &thread : threads)
	{
		thread.join();
	}
	async_log::flush(std::chrono::seconds(5));
	std::cout.rdbuf(original);

	std::istringstream lines(sink.str());
	std::string line;
	int count = 0;
	bool intact = true;
	while (std::getline(lines, line))
	{
		++count;
		intact = intact && (0 == line.rfind("thread", 0)) && (line.size() > 4) && (0 == line.compare(line.size() - 4, 4, " end"));
	}
	check(count == 800, "every line is written");
	check(intact, "no line is mixed with another thread's output");
}

static void test_stuck_sink_does_not_block_callers()
{
	std::printf("test_stuck_sink_does_not_block_callers\n");

	BlockingSink sink;
	auto *original = std::cout.rdbuf(&sink);

	const auto start = std::chrono::steady_clock::now();
	for (int i = 0; i < 100; ++i)
	{
		async_log::stream() << "queued " << i << std::endl;
	}
	const auto elapsed = std::chrono::steady_clock::now() - start;
	check(elapsed < std::chrono::seconds(1), "logging returns while the sink is stuck");
	check(!async_log::flush(std::chrono::milliseconds(50)), "flush() times out while the sink is stuck");

	sink.release.store(true);
	check(async_log::flush(std::chrono::seconds(2)), "flush() succeeds once the sink recovers");
	std::cout.rdbuf(original);

	check(sink.str().find("queued 99\n") != std::string::npos, "the backlog is written after recovery");
}

int main()
{
	test_lines_reach_cout_in_order();
	test_partial_line_waits_for_newline();
	test_threads_do_not_interleave_lines();
	test_stuck_sink_does_not_block_callers();

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("All async log tests passed\n");
	return 0;
}
