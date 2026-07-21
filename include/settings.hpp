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

	/// @brief Check whether Hydlift Aux-N integration is enabled.
	bool is_hydlift_aux_n_enabled() const;

	/// @brief Enable or disable Hydlift Aux-N integration.
	bool set_hydlift_aux_n_enabled(bool enabled, bool save = true);

	/// @brief Check whether incoming NMEA messages are enabled.
	bool is_nmea_read_enabled() const;

	/// @brief Enable or disable incoming NMEA messages.
	bool set_nmea_read_enabled(bool enabled, bool save = true);

	/// @brief Check whether cyclic NMEA messages are transmitted.
	bool is_nmea_send_enabled() const;

	/// @brief Enable or disable cyclic NMEA message transmission.
	bool set_nmea_send_enabled(bool enabled, bool save = true);

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
	 * @brief Check if the Virtual Terminal client UI is enabled
	 * @return True if the VT UI is enabled, false otherwise
	 */
	bool is_vt_enabled() const;

	/**
	 * @brief Set the Virtual Terminal client UI enabled state
	 * @param enabled Whether to enable the VT UI
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_vt_enabled(bool enabled, bool save = true);

	/**
	 * @brief Get the configured TC version
	 * @return The configured version (0-4, default 3)
	 */
	std::uint8_t get_tc_version() const;

	/**
	 * @brief Set the TC version
	 * @param version The version to set (0=DIS, 1=FDIS.1, 2=FirstEdition, 3=SecondEditionDraft, 4=SecondPublishedEdition)
	 * @param save Whether or not to save the settings to file
	 * @return True if the version was set successfully, false otherwise
	 */
	bool set_tc_version(std::uint8_t version, bool save = true);

	/**
	 * @brief Get the configured language code
	 * @return The language code (default "en")
	 */
	std::string get_language_code() const;

	/**
	 * @brief Set the language code
	 * @param code The language code
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_language_code(std::string code, bool save = true);

	/**
	 * @brief Get the configured country code
	 * @return The country code (default "US")
	 */
	std::string get_country_code() const;

	/**
	 * @brief Set the country code
	 * @param code The country code
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_country_code(std::string code, bool save = true);

	/**
	 * @brief Get the absolute path to the settings file
	 * @param filename The filename to get the path for
	 * @return The absolute path to the settings file
	 */
	static std::string get_filename_path(std::string);

private:
	bool set_boolean(bool &setting, bool enabled, bool save);

	constexpr static std::array<std::uint8_t, 3> DEFAULT_SUBNET = { 192, 168, 5 };
	constexpr static bool DEFAULT_TECU_ENABLED = true;
	constexpr static bool DEFAULT_HYDLIFT_AUX_N_ENABLED = false;
	constexpr static bool DEFAULT_NMEA_READ_ENABLED = false;
	constexpr static bool DEFAULT_NMEA_SEND_ENABLED = true;
	constexpr static bool DEFAULT_AOG_HEARTBEAT_ENABLED = true;
	constexpr static bool DEFAULT_VT_ENABLED = true;
	constexpr static std::uint8_t DEFAULT_TC_VERSION = 3; ///< SecondEditionDraft (V3 default for maximum implement compatibility)
	static const std::string DEFAULT_LANGUAGE_CODE;
	static const std::string DEFAULT_COUNTRY_CODE;
	std::array<std::uint8_t, 3> configuredSubnet = DEFAULT_SUBNET;
	bool tecuEnabled = DEFAULT_TECU_ENABLED;
	bool hydliftAuxNEnabled = DEFAULT_HYDLIFT_AUX_N_ENABLED;
	bool nmeaReadEnabled = DEFAULT_NMEA_READ_ENABLED;
	bool nmeaSendEnabled = DEFAULT_NMEA_SEND_ENABLED;
	bool aogHeartbeatEnabled = DEFAULT_AOG_HEARTBEAT_ENABLED;
	bool vtEnabled = DEFAULT_VT_ENABLED;
	std::uint8_t tcVersion = DEFAULT_TC_VERSION;
	std::string languageCode = DEFAULT_LANGUAGE_CODE;
	std::string countryCode = DEFAULT_COUNTRY_CODE;
};
