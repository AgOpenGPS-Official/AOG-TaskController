/**
 * @brief Unit tests for AsyncStreambuf, the wrapper that keeps a slow log sink off the main loop.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "async_streambuf.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <sstream>
#include <streambuf>
#include <thread>

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

// A sink that blocks every write until released, like a pipe nobody is reading.
class BlockingSink : public std::streambuf
{
public:
	std::atomic<bool> release{ false };
	std::atomic<int> writes{ 0 };

protected:
	std::streamsize xsputn(const char *, std::streamsize count) override
	{
		while (!release.load())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		++writes;
		return count;
	}

	int overflow(int ch) override
	{
		return ch;
	}
};

static void test_drains_everything_including_unflushed_tail()
{
	std::printf("test_drains_everything_including_unflushed_tail\n");

	std::stringbuf sink;
	std::ostream stream(&sink);
	{
		AsyncStreambuf async(stream);
		stream << "flushed line" << std::endl;
		stream << "unflushed tail\n";
		check(async.stop(std::chrono::seconds(2)), "stop() reports the drain finished");
	}
	check(sink.str() == "flushed line\nunflushed tail\n", "flushed and unflushed output both reach the sink");
	check(stream.rdbuf() == &sink, "the original streambuf is restored");
}

static void test_stop_is_idempotent()
{
	std::printf("test_stop_is_idempotent\n");

	std::stringbuf sink;
	std::ostream stream(&sink);
	AsyncStreambuf async(stream);
	stream << "once" << std::endl;
	check(async.stop(std::chrono::seconds(2)), "first stop() succeeds");
	check(async.stop(std::chrono::seconds(2)), "second stop() also succeeds");
}

static void test_stuck_sink_is_abandoned_safely()
{
	std::printf("test_stuck_sink_is_abandoned_safely\n");

	// Deliberately leaked: an abandoned drain thread keeps using it, exactly as the contract in
	// async_streambuf.hpp says the caller must allow for.
	auto *sink = new BlockingSink();
	std::ostream stream(sink);
	{
		auto async = std::make_unique<AsyncStreambuf>(stream);
		stream << "never delivered in time" << std::endl;

		const auto start = std::chrono::steady_clock::now();
		const bool drained = async->stop(std::chrono::milliseconds(100));
		const auto elapsed = std::chrono::steady_clock::now() - start;

		check(!drained, "stop() reports the stuck sink");
		check(elapsed < std::chrono::seconds(2), "stop() does not hang on a stuck sink");
		// The wrapper is destroyed while its drain thread is still blocked in the sink.
	}
	check(stream.rdbuf() == sink, "the original streambuf is restored");

	// Unblock the abandoned thread; it must finish using its own state without touching the
	// destroyed wrapper.
	sink->release.store(true);
	for (int i = 0; i < 1000 && sink->writes.load() == 0; ++i)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	check(sink->writes.load() == 1, "the abandoned thread completes its write after the wrapper is gone");
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

int main()
{
	test_drains_everything_including_unflushed_tail();
	test_stop_is_idempotent();
	test_stuck_sink_is_abandoned_safely();

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("All async streambuf tests passed\n");
	return 0;
}
