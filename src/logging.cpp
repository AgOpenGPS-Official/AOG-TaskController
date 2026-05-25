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
		std::lock_guard<std::mutex> lock(getLoggingMutex());
		if (c != EOF)
		{
			consoleBuffer->sputc(c); // Write to console
			fileStream.put(c); // Write to file
		}
		return c;
	}

	int sync() override
	{
		std::lock_guard<std::mutex> lock(getLoggingMutex());
		consoleBuffer->pubsync();
		fileStream.flush();
		return 0;
	}
};

static std::unique_ptr<TeeStreambuf> teeStream;

static void setup_file_logging()
{
	// Generate timestamped filename
	std::time_t now = std::time(nullptr);
	std::tm localTime;
	localtime_s(&localTime, &now); // Thread-safe localtime

	std::string logFilename = "logs\\AOG-TaskController_" +
	  std::to_string(localTime.tm_year + 1900) + "-" +
	  std::to_string(localTime.tm_mon + 1) + "-" +
	  std::to_string(localTime.tm_mday) + "_" +
	  std::to_string(localTime.tm_hour) + "-" +
	  std::to_string(localTime.tm_min) + ".log";

	teeStream = std::make_unique<TeeStreambuf>(std::cout, Settings::get_filename_path(logFilename));
}

// A log sink for the CAN stack
class CustomLogger : public isobus::CANStackLogger
{
public:
	/// @brief Destructor for the custom logger.
	virtual ~CustomLogger() = default;

	void sink_CAN_stack_log(CANStackLogger::LoggingLevel level, const std::string &text) override
	{
		std::lock_guard<std::mutex> lock(getLoggingMutex());
		std::cout << "[" << get_timestamp() << "] ";
		switch (level)
		{
			case LoggingLevel::Debug:
			{
				std::cout << "[Debug]";
			}
			break;

			case LoggingLevel::Info:
			{
				std::cout << "[Info]";
			}
			break;

			case LoggingLevel::Warning:
			{
				std::cout << "[Warn]";
			}
			break;

			case LoggingLevel::Error:
			{
				std::cout << "[Error]";
			}
			break;

			case LoggingLevel::Critical:
			{
				std::cout << "[Critical]";
			}
			break;
		}
		std::cout << text << std::endl; // Write the text to stdout
	}
};

static CustomLogger logger;
