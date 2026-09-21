#include "async_log.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "logging_utils.hpp"

#include <fstream>
#include <iostream>
#include <settings.hpp>

class TeeStreambuf : public std::streambuf
{
private:
	std::streambuf *consoleBuffer;
	std::ofstream fileStream;

public:
	TeeStreambuf(std::ostream &stream, const std::string &filename) :
	  consoleBuffer(stream.rdbuf()), fileStream(filename, std::ios::app)
	{
		std::cout << "Logging to file: " << filename << std::endl;
		stream.rdbuf(this);
	}

	~TeeStreambuf()
	{
		std::cout.rdbuf(consoleBuffer); // Restore original buffer
	}

protected:
	int overflow(int c) override
	{
		if (c != EOF)
		{
			consoleBuffer->sputc(c); // Write to console
			fileStream.put(c); // Write to file
		}
		return c;
	}

	int sync() override
	{
		consoleBuffer->pubsync();
		fileStream.flush();
		return 0;
	}
};

// Never deleted: the async log writer thread can still be inside it when the process exits.
static TeeStreambuf *teeStream = nullptr;

static void setup_file_logging()
{
	// Generate timestamped filename
	std::time_t now = std::time(nullptr);
	std::tm localTime;
#if defined(_WIN32)
	localtime_s(&localTime, &now); // Thread-safe localtime (Windows)
#else
	localtime_r(&now, &localTime); // Thread-safe localtime (POSIX)
#endif

	// Use a forward slash; Settings::get_filename_path will create the directory.
	std::string logFilename = std::string("logs/AOG-TaskController_") +
	  std::to_string(localTime.tm_year + 1900) + "-" +
	  std::to_string(localTime.tm_mon + 1) + "-" +
	  std::to_string(localTime.tm_mday) + "_" +
	  std::to_string(localTime.tm_hour) + "-" +
	  std::to_string(localTime.tm_min) + ".log";

	teeStream = new TeeStreambuf(std::cout, Settings::get_filename_path(logFilename));
}

// A log sink for the CAN stack
class CustomLogger : public isobus::CANStackLogger
{
public:
	/// @brief Destructor for the custom logger.
	virtual ~CustomLogger() = default;

	void sink_CAN_stack_log(CANStackLogger::LoggingLevel level, const std::string &text) override
	{
		std::ostream &out = async_log::stream();
		out << "[" << get_timestamp() << "] ";
		switch (level)
		{
			case LoggingLevel::Debug:
			{
				out << "[Debug]";
			}
			break;

			case LoggingLevel::Info:
			{
				out << "[Info]";
			}
			break;

			case LoggingLevel::Warning:
			{
				out << "[Warn]";
			}
			break;

			case LoggingLevel::Error:
			{
				out << "[Error]";
			}
			break;

			case LoggingLevel::Critical:
			{
				out << "[Critical]";
			}
			break;
		}
		out << text << std::endl;
	}
};

static CustomLogger logger;
