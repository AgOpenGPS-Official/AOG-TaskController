/**
 * @author Qoder
 * @brief GNSS receiver that parses J1939 PGNs from a John Deere SF3000 and generates $PANDA NMEA sentences
 * @version 0.1
 * @date 2025-03-14
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/utility/event_dispatcher.hpp"

#include "udp_connections.hpp"

class GnssReceiver
{
public:
	GnssReceiver();
	~GnssReceiver();

	/// @brief Register the CAN frame listener. Must be called after CANHardwareInterface::start().
	void initialize();

	/// @brief Build and send a $PANDA sentence if position and time data are available. Throttled to 10 Hz.
	/// @param udp The UDP connections to send the sentence on
	void send_panda_if_ready(std::shared_ptr<UdpConnections> udp);

private:
	struct GnssData
	{
		double latitude_deg = 0.0;
		double longitude_deg = 0.0;
		double altitude_m = 0.0;
		double heading_deg = 0.0;
		double speed_kmh = 0.0;

		std::uint8_t hour = 0;
		std::uint8_t minute = 0;
		std::uint16_t minute_ms = 0; ///< Milliseconds within the current minute

		bool has_position = false;
		bool has_time = false;
		bool has_altitude = false;
		bool has_heading = false;
		bool has_speed = false;
	};

	void on_can_frame(const isobus::CANMessageFrame &frame);
	void parse_position(const isobus::CANMessageFrame &frame);
	void parse_time_date(const isobus::CANMessageFrame &frame);
	void parse_altitude(const isobus::CANMessageFrame &frame);
	void parse_heading(const isobus::CANMessageFrame &frame);
	void parse_speed(const isobus::CANMessageFrame &frame);

	/// @brief Build the $PANDA sentence from the current GNSS data snapshot
	/// @param snapshot A copy of the current GNSS data (no lock needed)
	/// @return The complete $PANDA sentence including checksum and CRLF
	std::string build_panda(const GnssData &snapshot) const;

	static constexpr std::uint8_t SF3000_SOURCE_ADDRESS = 0x1C;

	GnssData data;
	std::mutex dataMutex;
	isobus::EventCallbackHandle canFrameHandle = 0;
	std::uint32_t lastPandaSend = 0;
	std::uint32_t lastPandaLog = 0;
	std::uint32_t lastWaitingLog = 0;
	bool pandaSending = false;
};
