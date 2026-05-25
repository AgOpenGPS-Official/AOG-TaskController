#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

// Utility function to get current timestamp string (HH:MM:SS.mmm)
inline std::string get_timestamp()
{
	auto now = std::chrono::system_clock::now();
	auto time_t_now = std::chrono::system_clock::to_time_t(now);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

	std::tm localTime;
#if defined(_WIN32)
	localtime_s(&localTime, &time_t_now);
#else
	localtime_r(&time_t_now, &localTime);
#endif

	std::ostringstream oss;
	oss << std::setfill('0') << std::setw(2) << localTime.tm_hour << ":"
	    << std::setfill('0') << std::setw(2) << localTime.tm_min << ":"
	    << std::setfill('0') << std::setw(2) << localTime.tm_sec << "."
	    << std::setfill('0') << std::setw(3) << ms.count();
	return oss.str();
}

// Global mutex to protect all std::cout writes from concurrent access across threads
inline std::mutex &getLoggingMutex()
{
	static std::mutex loggingMutex;
	return loggingMutex;
}
