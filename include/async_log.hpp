#pragma once

#include <chrono>
#include <ostream>

// Log output that can't stall the calling thread. Application::update() also drives the ISOBUS
// transmit path, so a log line blocked on a slow console pipe or disk would delay periodic sends.
namespace async_log
{
	/// This thread's log stream. Each line (up to its '\n') is queued and written to std::cout by a
	/// background thread.
	std::ostream &stream();

	/// Waits until everything queued so far has been written to std::cout.
	/// Returns false if the timeout ran out first (for example, the console is stuck).
	bool flush(std::chrono::milliseconds timeout);
} // namespace async_log
