/**
 * @author Daan Steenbergen
 * @brief An interface to store/load AOG-TC settings to/from a file
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>

/// @brief A class to store/load AOG-TC settings to/from a file
class Settings
{
public:
	/**
     * @brief (re)load the settings from specified file
     * @return True if the settings were loaded successfully, false otherwise
     */
	bool load();

	/**
     * @brief Save the settings to specified file
     * @return True if the settings were saved successfully, false otherwise
     */
	bool save() const;

	/**
	 * @brief Get the configured subnet
	 * @return The configured subnet
	 */
	const std::array<std::uint8_t, 3> &get_subnet() const;

	/**
	 * @brief Get the configured subnet as a string
	 * @return The configured subnet as a string
	 */
	std::string get_subnet_string() const;

	/**
	 * @brief Set the configured subnet
	 * @param subnet The subnet to set
	 * @param save Whether or not to save the settings to file
	 * @return True if the subnet was set successfully, false otherwise
	 */
	bool set_subnet(std::array<std::uint8_t, 3> subnet, bool save = true);

	/**
	 * @brief Check if Tractor ECU is enabled
	 * @return True if TECU is enabled, false otherwise
	 */
	bool is_tecu_enabled() const;

	/**
	 * @brief Set the Tractor ECU enabled state
	 * @param enabled Whether to enable the Tractor ECU
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_tecu_enabled(bool enabled, bool save = true);

	/**
	 * @brief Check if AOG heartbeat is enabled
	 * @return True if heartbeat is enabled, false otherwise
	 */
	bool is_aog_heartbeat_enabled() const;

	/**
	 * @brief Set the AOG heartbeat enabled state
	 * @param enabled Whether to enable the AOG heartbeat
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_aog_heartbeat_enabled(bool enabled, bool save = true);

	/**
	 * @brief Get the configured TC ISO 11783-10 version
	 * @return The configured version (0-4, default 4)
	 */
	std::uint8_t get_tc_version() const;

	/**
	 * @brief Set the TC ISO 11783-10 version
	 * @param version The version to set (0=DIS, 1=FDIS.1, 2=FirstEdition, 3=SecondEditionDraft, 4=SecondPublishedEdition)
	 * @param save Whether or not to save the settings to file
	 * @return True if the version was set successfully, false otherwise
	 */
	bool set_tc_version(std::uint8_t version, bool save = true);

	/**
	 * @brief Get the absolute path to the settings file
	 * @param filename The filename to get the path for
	 * @return The absolute path to the settings file
	 */
	static std::string get_filename_path(std::string);

	/**
	 * @brief Get the configured language code (ISO 639-1)
	 * @return The language code (default "en")
	 */
	std::string get_language_code() const;

	/**
	 * @brief Set the language code
	 * @param code The ISO 639-1 language code
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_language_code(std::string code, bool save = true);

	/**
	 * @brief Get the configured country code (ISO 3166-1 alpha-2)
	 * @return The country code (default "US")
	 */
	std::string get_country_code() const;

	/**
	 * @brief Set the country code
	 * @param code The ISO 3166-1 alpha-2 country code
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_country_code(std::string code, bool save = true);

private:
	constexpr static std::array<std::uint8_t, 3> DEFAULT_SUBNET = { 192, 168, 5 };
	constexpr static bool DEFAULT_TECU_ENABLED = true;
	constexpr static bool DEFAULT_AOG_HEARTBEAT_ENABLED = true;
	constexpr static std::uint8_t DEFAULT_TC_VERSION = 3; // SecondEditionDraft (V3 default for maximum implement compatibility)
	static const std::string DEFAULT_LANGUAGE_CODE;
	static const std::string DEFAULT_COUNTRY_CODE;
	std::array<std::uint8_t, 3> configuredSubnet = DEFAULT_SUBNET;
	bool tecuEnabled = DEFAULT_TECU_ENABLED;
	bool aogHeartbeatEnabled = DEFAULT_AOG_HEARTBEAT_ENABLED;
	std::uint8_t tcVersion = DEFAULT_TC_VERSION;
	std::string languageCode = DEFAULT_LANGUAGE_CODE;
	std::string countryCode = DEFAULT_COUNTRY_CODE;
};
