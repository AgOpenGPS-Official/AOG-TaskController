#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

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

inline std::ostream &log()
{
	return std::cout << "[" << get_timestamp() << "] ";
}

inline std::ostream &log(const std::string &tag)
{
	return log() << "[" << tag << "] ";
}
