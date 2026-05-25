/**
 * @author Daan Steenbergen
 * @brief An interface to store/load AOG-TC settings to/from a file
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "settings.hpp"
#include "logging_utils.hpp"

#include <ShlObj_core.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

const std::string Settings::DEFAULT_LANGUAGE_CODE = "en";
const std::string Settings::DEFAULT_COUNTRY_CODE = "US";

bool Settings::load()
{
	std::ifstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		return false;
	}

	json data;
	file >> data;

	if (data.contains("subnet"))
	{
		try
		{
			auto subnetData = data["subnet"].get<std::array<int, 3>>(); // Directly get the array
			std::copy(subnetData.begin(), subnetData.end(), configuredSubnet.begin());
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'subnet': " << e.what() << std::endl;
			configuredSubnet = DEFAULT_SUBNET; // Fallback to default
		}
	}
	else
	{
		configuredSubnet = DEFAULT_SUBNET; // Key not found, use default
	}

	if (data.contains("tecuEnabled"))
	{
		try
		{
			tecuEnabled = data["tecuEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'tecuEnabled': " << e.what() << std::endl;
			tecuEnabled = DEFAULT_TECU_ENABLED; // Fallback to default
		}
	}
	else
	{
		tecuEnabled = DEFAULT_TECU_ENABLED; // Key not found, use default
	}

	if (data.contains("aogHeartbeatEnabled"))
	{
		try
		{
			aogHeartbeatEnabled = data["aogHeartbeatEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'aogHeartbeatEnabled': " << e.what() << std::endl;
			aogHeartbeatEnabled = DEFAULT_AOG_HEARTBEAT_ENABLED; // Fallback to default
		}
	}
	else
	{
		aogHeartbeatEnabled = DEFAULT_AOG_HEARTBEAT_ENABLED; // Key not found, use default
	}

	if (data.contains("tcVersion"))
	{
		try
		{
			int version = data["tcVersion"].get<int>();
			if (version >= 0 && version <= 4)
			{
				tcVersion = static_cast<std::uint8_t>(version);
			}
			else
			{
				std::cout << "[" << get_timestamp() << "] Invalid tcVersion " << version << ", using default " << static_cast<int>(DEFAULT_TC_VERSION) << std::endl;
				tcVersion = DEFAULT_TC_VERSION;
			}
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'tcVersion': " << e.what() << std::endl;
			tcVersion = DEFAULT_TC_VERSION; // Fallback to default
		}
	}
	else
	{
		tcVersion = DEFAULT_TC_VERSION; // Key not found, use default
	}

	if (data.contains("languageCode"))
	{
		try
		{
			languageCode = data["languageCode"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'languageCode': " << e.what() << std::endl;
			languageCode = DEFAULT_LANGUAGE_CODE;
		}
	}
	else
	{
		languageCode = DEFAULT_LANGUAGE_CODE;
	}

	if (data.contains("countryCode"))
	{
		try
		{
			countryCode = data["countryCode"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing 'countryCode': " << e.what() << std::endl;
			countryCode = DEFAULT_COUNTRY_CODE;
		}
	}
	else
	{
		countryCode = DEFAULT_COUNTRY_CODE;
	}

	return true;
}

bool Settings::save() const
{
	json data;
	data["subnet"] = configuredSubnet;
	data["tecuEnabled"] = tecuEnabled;
	data["aogHeartbeatEnabled"] = aogHeartbeatEnabled;
	data["tcVersion"] = tcVersion;
	data["languageCode"] = languageCode;
	data["countryCode"] = countryCode;

	std::ofstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		return false;
	}

	file << data.dump(4); // Pretty print
	return true;
}

std::uint8_t Settings::get_tc_version() const
{
	return tcVersion;
}

bool Settings::set_tc_version(std::uint8_t version, bool save)
{
	if (version > 4)
	{
		std::cout << "[" << get_timestamp() << "] Invalid TC version " << static_cast<int>(version) << ", using default " << static_cast<int>(DEFAULT_TC_VERSION) << std::endl;
		tcVersion = DEFAULT_TC_VERSION;
	}
	else
	{
		tcVersion = version;
	}
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_language_code() const
{
	return languageCode;
}

bool Settings::set_language_code(std::string code, bool save)
{
	languageCode = code;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_country_code() const
{
	return countryCode;
}

bool Settings::set_country_code(std::string code, bool save)
{
	countryCode = code;
	if (save)
	{
		return this->save();
	}
	return true;
}

const std::array<std::uint8_t, 3> &Settings::get_subnet() const
{
	return configuredSubnet;
}

std::string Settings::get_subnet_string() const
{
	return std::to_string(configuredSubnet[0]) + '.' + std::to_string(configuredSubnet[1]) + '.' + std::to_string(configuredSubnet[2]) + ".0";
}

bool Settings::set_subnet(std::array<std::uint8_t, 3> subnet, bool save)
{
	configuredSubnet = subnet;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_tecu_enabled() const
{
	return tecuEnabled;
}

bool Settings::set_tecu_enabled(bool enabled, bool save)
{
	tecuEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_aog_heartbeat_enabled() const
{
	return aogHeartbeatEnabled;
}

bool Settings::set_aog_heartbeat_enabled(bool enabled, bool save)
{
	aogHeartbeatEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_filename_path(std::string fileName)
{
	char path[MAX_PATH];
	if (SHGetFolderPath(NULL, CSIDL_APPDATA, NULL, 0, path) != S_OK)
	{
		throw std::runtime_error("Failed to get AppData path");
	}

	std::string baseDir = std::string(path) + "\\" + PROJECT_NAME;
	std::string fullPath = baseDir + "\\" + fileName;

	// Find the last directory separator (before the actual file name)
	size_t lastSlash = fullPath.find_last_of("\\/");
	if (lastSlash != std::string::npos)
	{
		std::string directoryPath = fullPath.substr(0, lastSlash); // Extract the directory part

		// Create each directory level iteratively
		std::istringstream dirStream(directoryPath);
		std::string segment;
		std::string currentPath;

		while (std::getline(dirStream, segment, '\\')) // Split by `\`
		{
			if (!currentPath.empty())
				currentPath += "\\"; // Append separator only after first segment

			currentPath += segment;

			if (CreateDirectory(currentPath.c_str(), NULL) == 0)
			{
				DWORD error = GetLastError();
				if (error != ERROR_ALREADY_EXISTS)
				{
					throw std::runtime_error("Failed to create directory: " + currentPath);
				}
			}
		}
	}

	return fullPath;
}
