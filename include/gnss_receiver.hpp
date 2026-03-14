/**
 * @author Qoder
 * @brief GNSS receiver that parses J1939 PGNs from a John Deere SF3000 and generates $PANDA NMEA sentences
 * @version 0.2
 * @date 2025-03-14
 */

#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

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
	/// @param reverseEngineerMode If true, log all SF3000 frames to CSV for offline analysis
	void initialize(bool reverseEngineerMode = false);

	/// @brief Build and send a $PANDA sentence if position data is available. Throttled to 10 Hz.
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
		std::uint16_t minute_ms = 0;

		bool has_position = false;
		bool has_time = false;
		bool has_altitude = false;
		bool has_heading = false;
		bool has_speed = false;
	};

	/// @brief Per-PGN tracking for reverse engineering
	struct PgnTracker
	{
		std::uint32_t count = 0;
		std::uint8_t lastPayload[8] = {};
		std::uint8_t lastLength = 0;
		std::uint32_t firstSeenMs = 0;
		std::uint32_t lastSeenMs = 0;
	};

	void on_can_frame(const isobus::CANMessageFrame &frame);

	// Standard J1939 parsers (kept for buses that broadcast them)
	void parse_position_standard(const isobus::CANMessageFrame &frame);
	void parse_time_date_standard(const isobus::CANMessageFrame &frame);
	void parse_altitude_standard(const isobus::CANMessageFrame &frame);
	void parse_speed_standard(const isobus::CANMessageFrame &frame);

	// Confirmed/candidate parsers for proprietary Deere PGNs
	void parse_heading_FE48(const isobus::CANMessageFrame &frame);
	void parse_candidate_FE45(const isobus::CANMessageFrame &frame);
	void parse_candidate_FE43(const isobus::CANMessageFrame &frame);
	void parse_candidate_FE12(const isobus::CANMessageFrame &frame);
	void parse_candidate_FE13(const isobus::CANMessageFrame &frame);
	void parse_candidate_FFFA(const isobus::CANMessageFrame &frame);
	void parse_candidate_FFFB(const isobus::CANMessageFrame &frame);
	void parse_candidate_ACFF(const isobus::CANMessageFrame &frame);
	void parse_candidate_FE0A(const isobus::CANMessageFrame &frame);
	void parse_candidate_F022(const isobus::CANMessageFrame &frame);
	void parse_candidate_FFFF(const isobus::CANMessageFrame &frame);

	void log_frame_raw(std::uint32_t pgn, std::uint8_t sa, const isobus::CANMessageFrame &frame);
	void log_frame_csv(std::uint32_t pgn, std::uint8_t sa, const isobus::CANMessageFrame &frame);

	std::string build_gga(const GnssData &snapshot) const;

	static constexpr std::uint8_t SF3000_SOURCE_ADDRESS = 0x9A;

	GnssData data;
	std::mutex dataMutex;
	isobus::EventCallbackHandle canFrameHandle = 0;
	std::uint32_t lastPandaSend = 0;
	std::uint32_t lastPandaLog = 0;
	std::uint32_t lastWaitingLog = 0;
	std::uint32_t lastPgnSummaryLog = 0;
	bool pandaSending = false;
	bool reMode = false;

	std::unordered_map<std::uint32_t, PgnTracker> pgnTrackers;
	std::mutex trackerMutex;
	std::ofstream csvFile;
};
