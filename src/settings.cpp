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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#if defined(_WIN32)
#include <ShlObj_core.h>
#include <Windows.h>
#endif

using json = nlohmann::json;

const std::string Settings::DEFAULT_LANGUAGE_CODE = "en";
const std::string Settings::DEFAULT_COUNTRY_CODE = "US";

bool Settings::load()
{
	std::scoped_lock lock(settingsMutex);
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

	auto loadBoolean = [&data](const char *key, bool defaultValue) {
		try
		{
			return data.value(key, defaultValue);
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "[" << get_timestamp() << "] Error parsing '" << key << "': " << e.what() << std::endl;
			return defaultValue;
		}
	};
	tecuEnabled = loadBoolean("tecuEnabled", DEFAULT_TECU_ENABLED);
	nmeaSendEnabled = loadBoolean("nmeaSendEnabled", DEFAULT_NMEA_SEND_ENABLED);
	aogHeartbeatEnabled = loadBoolean("aogHeartbeatEnabled", DEFAULT_AOG_HEARTBEAT_ENABLED);
	vtEnabled = loadBoolean("vtEnabled", DEFAULT_VT_ENABLED);

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
			tcVersion = DEFAULT_TC_VERSION;
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
	std::scoped_lock lock(settingsMutex);
	json data;
	data["subnet"] = configuredSubnet;
	data["tecuEnabled"] = tecuEnabled;
	data["nmeaSendEnabled"] = nmeaSendEnabled;
	data["aogHeartbeatEnabled"] = aogHeartbeatEnabled;
	data["vtEnabled"] = vtEnabled;
	data["tcVersion"] = tcVersion;
	data["languageCode"] = languageCode;
	data["countryCode"] = countryCode;

	const std::filesystem::path settingsPath(get_filename_path("settings.json"));
	std::filesystem::path temporaryPath = settingsPath;
	temporaryPath += ".tmp";

	std::ofstream file(temporaryPath, std::ios::out | std::ios::trunc);
	if (!file.is_open())
	{
		return false;
	}

	file << data.dump(4); // Pretty print
	file.flush();
	const bool writeSucceeded = file.good();
	file.close();
	if (!writeSucceeded || file.fail())
	{
		std::error_code cleanupError;
		std::filesystem::remove(temporaryPath, cleanupError);
		return false;
	}

#if defined(_WIN32)
	if (!MoveFileExW(
	      temporaryPath.c_str(),
	      settingsPath.c_str(),
	      MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
	{
		std::error_code cleanupError;
		std::filesystem::remove(temporaryPath, cleanupError);
		return false;
	}
#else
	std::error_code renameError;
	std::filesystem::rename(temporaryPath, settingsPath, renameError);
	if (renameError)
	{
		std::error_code cleanupError;
		std::filesystem::remove(temporaryPath, cleanupError);
		return false;
	}
#endif

	return true;
}

std::array<std::uint8_t, 3> Settings::get_subnet() const
{
	std::scoped_lock lock(settingsMutex);
	return configuredSubnet;
}

std::string Settings::get_subnet_string() const
{
	std::scoped_lock lock(settingsMutex);
	return std::to_string(configuredSubnet[0]) + '.' + std::to_string(configuredSubnet[1]) + '.' + std::to_string(configuredSubnet[2]) + ".0";
}

bool Settings::set_subnet(std::array<std::uint8_t, 3> subnet, bool save)
{
	std::scoped_lock lock(settingsMutex);
	configuredSubnet = subnet;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_tecu_enabled() const
{
	std::scoped_lock lock(settingsMutex);
	return tecuEnabled;
}

bool Settings::set_tecu_enabled(bool enabled, bool save)
{
	return set_boolean(tecuEnabled, enabled, save);
}

bool Settings::is_nmea_send_enabled() const
{
	std::scoped_lock lock(settingsMutex);
	return nmeaSendEnabled;
}

bool Settings::set_nmea_send_enabled(bool enabled, bool save)
{
	return set_boolean(nmeaSendEnabled, enabled, save);
}

bool Settings::set_boolean(bool &setting, bool enabled, bool save)
{
	std::scoped_lock lock(settingsMutex);
	const bool previousValue = setting;
	setting = enabled;
	if (!save || this->save())
	{
		return true;
	}
	setting = previousValue;
	return false;
}

bool Settings::is_vt_enabled() const
{
	std::scoped_lock lock(settingsMutex);
	return vtEnabled;
}

bool Settings::set_vt_enabled(bool enabled, bool save)
{
	return set_boolean(vtEnabled, enabled, save);
}

bool Settings::is_aog_heartbeat_enabled() const
{
	std::scoped_lock lock(settingsMutex);
	return aogHeartbeatEnabled;
}

bool Settings::set_aog_heartbeat_enabled(bool enabled, bool save)
{
	return set_boolean(aogHeartbeatEnabled, enabled, save);
}

std::uint8_t Settings::get_tc_version() const
{
	std::scoped_lock lock(settingsMutex);
	return tcVersion;
}

bool Settings::set_tc_version(std::uint8_t version, bool save)
{
	std::scoped_lock lock(settingsMutex);
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
	std::scoped_lock lock(settingsMutex);
	return languageCode;
}

bool Settings::set_language_code(std::string code, bool save)
{
	std::scoped_lock lock(settingsMutex);
	languageCode = code;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_country_code() const
{
	std::scoped_lock lock(settingsMutex);
	return countryCode;
}

bool Settings::set_country_code(std::string code, bool save)
{
	std::scoped_lock lock(settingsMutex);
	countryCode = code;
	if (save)
	{
		return this->save();
	}
	return true;
}

namespace
{
	std::filesystem::path get_base_config_dir()
	{
#if defined(_WIN32)
		// Use the same call the original Windows build linked against, so we
		// don't take a new ole32 dependency. CSIDL_APPDATA == FOLDERID_RoamingAppData.
		char path[MAX_PATH] = { 0 };
		if (SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, path) != S_OK)
		{
			throw std::runtime_error("Failed to get AppData path");
		}
		return std::filesystem::path(path) / PROJECT_NAME;
#elif defined(__APPLE__)
		const char *home = std::getenv("HOME");
		if (!home)
		{
			throw std::runtime_error("HOME environment variable not set");
		}
		return std::filesystem::path(home) / "Library" / "Application Support" / PROJECT_NAME;
#else
		// Linux / other Unix: XDG Base Directory specification
		const char *xdg = std::getenv("XDG_CONFIG_HOME");
		if (xdg && *xdg)
		{
			return std::filesystem::path(xdg) / PROJECT_NAME;
		}
		const char *home = std::getenv("HOME");
		if (!home)
		{
			throw std::runtime_error("HOME environment variable not set");
		}
		return std::filesystem::path(home) / ".config" / PROJECT_NAME;
#endif
	}
}

std::string Settings::get_filename_path(std::string fileName)
{
	auto basePath = get_base_config_dir();
	auto fullPath = basePath / fileName;

	std::error_code ec;
	std::filesystem::create_directories(fullPath.parent_path(), ec);
	if (ec)
	{
		throw std::runtime_error("Failed to create config directory " + fullPath.parent_path().string() + ": " + ec.message());
	}

	return fullPath.string();
}
