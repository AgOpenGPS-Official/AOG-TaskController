/**
 * @author Qoder
 * @brief GNSS receiver that parses J1939 PGNs from a John Deere SF3000 and generates $PANDA NMEA sentences
 * @version 0.2
 * @date 2025-03-14
 */

#include "gnss_receiver.hpp"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "isobus/utility/system_timing.hpp"

GnssReceiver::GnssReceiver()
{
}

GnssReceiver::~GnssReceiver()
{
	isobus::CANHardwareInterface::get_can_frame_received_event_dispatcher()
	  .remove_listener(canFrameHandle);
	if (csvFile.is_open())
	{
		csvFile.close();
	}
}

void GnssReceiver::initialize(bool reverseEngineerMode)
{
	reMode = reverseEngineerMode;

	if (reMode)
	{
		csvFile.open("sf3000_can_log.csv", std::ios::out | std::ios::trunc);
		if (csvFile.is_open())
		{
			csvFile << "timestamp_ms,can_id,pgn,sa,dlc,b0,b1,b2,b3,b4,b5,b6,b7" << std::endl;
			std::cout << "[GnssReceiver] RE mode: logging all SA=0x"
			          << std::hex << static_cast<int>(SF3000_SOURCE_ADDRESS) << std::dec
			          << " frames to sf3000_can_log.csv" << std::endl;
		}
	}

	canFrameHandle = isobus::CANHardwareInterface::get_can_frame_received_event_dispatcher()
	                   .add_listener([this](const isobus::CANMessageFrame &frame) {
		                   on_can_frame(frame);
	                   });
	std::cout << "[GnssReceiver] CAN frame listener registered for SF3000 (SA=0x"
	          << std::hex << static_cast<int>(SF3000_SOURCE_ADDRESS) << std::dec << ")" << std::endl;
}

void GnssReceiver::log_frame_raw(std::uint32_t pgn, std::uint8_t sa, const isobus::CANMessageFrame &frame)
{
	std::ostringstream oss;
	oss << "[GnssReceiver] CAN ID=0x" << std::hex << std::uppercase << frame.identifier
	    << " PGN=0x" << pgn << " SA=0x" << static_cast<int>(sa)
	    << " DLC=" << std::dec << static_cast<int>(frame.dataLength) << " DATA=[";
	for (int i = 0; i < frame.dataLength; i++)
	{
		if (i > 0)
			oss << " ";
		oss << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(frame.data[i]);
	}
	oss << "]" << std::dec;
	std::cout << oss.str() << std::endl;
}

void GnssReceiver::log_frame_csv(std::uint32_t pgn, std::uint8_t sa, const isobus::CANMessageFrame &frame)
{
	if (!csvFile.is_open())
		return;

	csvFile << isobus::SystemTiming::get_timestamp_ms() << ","
	        << "0x" << std::hex << std::uppercase << frame.identifier << ","
	        << "0x" << pgn << ","
	        << "0x" << static_cast<int>(sa) << std::dec << ","
	        << static_cast<int>(frame.dataLength);
	for (int i = 0; i < 8; i++)
	{
		csvFile << ",";
		if (i < frame.dataLength)
			csvFile << "0x" << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(frame.data[i]) << std::dec;
	}
	csvFile << std::endl;
}

void GnssReceiver::on_can_frame(const isobus::CANMessageFrame &frame)
{
	if (!frame.isExtendedFrame)
		return;

	std::uint8_t sa = frame.identifier & 0xFF;
	std::uint8_t pf = (frame.identifier >> 16) & 0xFF;

	std::uint32_t pgn;
	if (pf >= 0xF0)
	{
		pgn = (frame.identifier >> 8) & 0x3FFFF;
	}
	else
	{
		pgn = (frame.identifier >> 8) & 0x3FF00;
	}

	// Track all frames from SF3000
	if (sa == SF3000_SOURCE_ADDRESS)
	{
		// Update PGN tracker
		{
			std::lock_guard<std::mutex> lock(trackerMutex);
			auto &tracker = pgnTrackers[pgn];
			if (tracker.count == 0)
			{
				tracker.firstSeenMs = isobus::SystemTiming::get_timestamp_ms();
				std::cout << "[GnssReceiver] NEW PGN discovered from SF3000: 0x"
				          << std::hex << std::uppercase << pgn << std::dec
				          << " (CAN ID=0x" << std::hex << frame.identifier << std::dec << ")" << std::endl;
				// Log first occurrence payload in detail
				log_frame_raw(pgn, sa, frame);
			}
			tracker.count++;
			tracker.lastSeenMs = isobus::SystemTiming::get_timestamp_ms();
			tracker.lastLength = frame.dataLength;
			for (int i = 0; i < frame.dataLength && i < 8; i++)
			{
				tracker.lastPayload[i] = frame.data[i];
			}
		}

		// CSV logging in RE mode
		if (reMode)
		{
			log_frame_csv(pgn, sa, frame);
		}

		// Dispatch to parsers for known/candidate PGNs
		switch (pgn)
		{
			// Standard J1939 PGNs (may or may not be present)
			case 0xFEF3:
				parse_position_standard(frame);
				break;
			case 0xFEF0:
				parse_time_date_standard(frame);
				break;
			case 0xFEF2:
				parse_altitude_standard(frame);
				break;

			// Confirmed candidate: heading
			case 0xFE45:
				parse_heading_FE45(frame);
				break;

			// Proprietary Deere PGNs - candidate parsers
			case 0xFE43:
				parse_candidate_FE43(frame);
				break;
			case 0xFE12:
				parse_candidate_FE12(frame);
				break;
			case 0xFE13:
				parse_candidate_FE13(frame);
				break;
			case 0xFE0A:
				parse_candidate_FE0A(frame);
				break;
			case 0xF022:
				parse_candidate_F022(frame);
				break;
			case 0xFFFA:
				parse_candidate_FFFA(frame);
				break;
			case 0xFFFB:
				parse_candidate_FFFB(frame);
				break;
			case 0xFFFF:
				parse_candidate_FFFF(frame);
				break;

			default:
				break;
		}
	}

	// Handle ACFF: PDU1 with PF=0xAC, destination=0xFF (global broadcast)
	// CAN ID example: 0x0CACFF9A -> PGN extracted as 0xAC00 by PDU1 rules (PS is destination, not part of PGN)
	if (sa == SF3000_SOURCE_ADDRESS && pgn == 0xAC00)
	{
		parse_candidate_ACFF(frame);
	}

	// Speed PGN from any source address (standard J1939)
	if (pgn == 0xFEF1)
	{
		parse_speed_standard(frame);
	}
}

// ============================================================================
// Standard J1939 parsers (kept for buses that may still broadcast them)
// ============================================================================

void GnssReceiver::parse_position_standard(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 8)
		return;

	std::uint32_t raw_lat =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);
	std::uint32_t raw_lon =
	  static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[6]) << 16) | (static_cast<std::uint32_t>(frame.data[7]) << 24);

	if (raw_lat == 0xFFFFFFFF || raw_lon == 0xFFFFFFFF)
		return;

	double lat = raw_lat * 1e-7 - 210.0;
	double lon = raw_lon * 1e-7 - 210.0;

	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
	{
		std::cout << "[GnssReceiver] PGN 0xFEF3 position out of range: lat=" << lat << " lon=" << lon << std::endl;
		return;
	}

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_position)
	{
		std::cout << "[GnssReceiver] *** POSITION set by PGN 0xFEF3 *** lat=" << lat << " lon=" << lon << std::endl;
	}
	data.latitude_deg = lat;
	data.longitude_deg = lon;
	data.has_position = true;
}

void GnssReceiver::parse_time_date_standard(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 4)
		return;

	std::uint16_t ms_of_minute = static_cast<std::uint16_t>(
	  static_cast<std::uint16_t>(frame.data[0]) | (static_cast<std::uint16_t>(frame.data[1]) << 8));
	std::uint8_t min = frame.data[2];
	std::uint8_t hr = frame.data[3];

	if (hr > 23 || min > 59 || ms_of_minute > 59999)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_time)
	{
		std::cout << "[GnssReceiver] *** TIME set by PGN 0xFEF0 *** "
		          << static_cast<int>(hr) << ":" << static_cast<int>(min) << " (" << ms_of_minute << "ms)" << std::endl;
	}
	data.hour = hr;
	data.minute = min;
	data.minute_ms = ms_of_minute;
	data.has_time = true;
}

void GnssReceiver::parse_altitude_standard(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 4)
		return;

	std::uint32_t raw_alt =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);

	if (raw_alt == 0xFFFFFFFF)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_altitude)
	{
		std::cout << "[GnssReceiver] *** ALTITUDE set by PGN 0xFEF2 *** " << (raw_alt * 0.01) << " m" << std::endl;
	}
	data.altitude_m = raw_alt * 0.01;
	data.has_altitude = true;
}

void GnssReceiver::parse_speed_standard(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 3)
		return;

	std::uint16_t raw_speed = static_cast<std::uint16_t>(
	  static_cast<std::uint16_t>(frame.data[1]) | (static_cast<std::uint16_t>(frame.data[2]) << 8));

	if (raw_speed == 0xFFFF)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_speed)
	{
		std::cout << "[GnssReceiver] *** SPEED set by PGN 0xFEF1 *** " << (raw_speed / 256.0) << " km/h (SA=0x"
		          << std::hex << static_cast<int>(frame.identifier & 0xFF) << std::dec << ")" << std::endl;
	}
	data.speed_kmh = raw_speed / 256.0;
	data.has_speed = true;
}

// ============================================================================
// Confirmed/candidate: PGN 0xFE45 - Heading
// ============================================================================

void GnssReceiver::parse_heading_FE45(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 2)
		return;

	std::uint16_t raw_heading = static_cast<std::uint16_t>(
	  static_cast<std::uint16_t>(frame.data[0]) | (static_cast<std::uint16_t>(frame.data[1]) << 8));

	if (raw_heading == 0xFFFF)
		return;

	// Deere proprietary scale: raw / 27.5 degrees (confirmed via RE capture)
	double heading = raw_heading / 27.5;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_heading)
	{
		std::cout << "[GnssReceiver] *** HEADING set by PGN 0xFE45 *** " << heading << " deg (raw=" << raw_heading << ")" << std::endl;
	}
	data.heading_deg = heading;
	data.has_heading = true;
}

// ============================================================================
// Proprietary Deere candidate parsers
// Each logs decoded candidate fields. Actual field mapping TBD via RE.
// ============================================================================

void GnssReceiver::parse_candidate_FE43(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFE43 (65091) - Deere proprietary, possibly vehicle dynamics or tilt
	// Expected: near Kecskemet, tilt = -1.2 deg
	if (frame.dataLength < 2)
		return;

	std::uint16_t w0 = static_cast<std::uint16_t>(frame.data[0]) | (static_cast<std::uint16_t>(frame.data[1]) << 8);
	double candidate_angle = static_cast<std::int16_t>(w0) * 0.0078125; // same scale as heading?

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFE43 candidate: word0_as_angle=" << candidate_angle
		          << " word0_raw=" << w0 << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FE12(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFE12 (65042) - Deere proprietary
	if (frame.dataLength < 4)
		return;

	std::uint32_t dw0 =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);

	// Try as J1939 position (uint32 * 1e-7 - 210)
	double as_pos = dw0 * 1e-7 - 210.0;
	// Try as altitude (uint32 * 0.01)
	double as_alt = dw0 * 0.01;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFE12 candidate: dw0=0x" << std::hex << dw0 << std::dec
		          << " as_position=" << as_pos
		          << " as_altitude_cm=" << as_alt << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FE13(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFE13 (65043) - Deere proprietary
	if (frame.dataLength < 4)
		return;

	std::uint32_t dw0 =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);

	double as_pos = dw0 * 1e-7 - 210.0;
	double as_alt = dw0 * 0.01;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFE13 candidate: dw0=0x" << std::hex << dw0 << std::dec
		          << " as_position=" << as_pos
		          << " as_altitude_cm=" << as_alt << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FE0A(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFE0A (65034) - Deere proprietary
	if (frame.dataLength < 2)
		return;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFE0A candidate: first payload logged at discovery" << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_F022(const isobus::CANMessageFrame &frame)
{
	// PGN 0xF022 (61474) - Deere proprietary, possibly GNSS status
	if (frame.dataLength < 2)
		return;

	// Byte 0 might be fix type, byte 1 might be satellite count
	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xF022 candidate: b0=" << static_cast<int>(frame.data[0])
		          << " b1=" << static_cast<int>(frame.data[1]);
		if (frame.dataLength >= 4)
		{
			std::uint16_t w1 = static_cast<std::uint16_t>(frame.data[2]) | (static_cast<std::uint16_t>(frame.data[3]) << 8);
			std::cout << " w1=" << w1 << " w1_as_hdop_0.01=" << (w1 * 0.01);
		}
		std::cout << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FFFA(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFFFA (65530) - Deere proprietary, possibly GNSS position or extended data
	if (frame.dataLength < 8)
		return;

	// Try interpreting as two uint32 position fields (same encoding as PGN 0xFEF3)
	std::uint32_t dw0 =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);
	std::uint32_t dw1 =
	  static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[6]) << 16) | (static_cast<std::uint32_t>(frame.data[7]) << 24);

	double as_lat = dw0 * 1e-7 - 210.0;
	double as_lon = dw1 * 1e-7 - 210.0;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFFFA candidate: dw0=0x" << std::hex << dw0
		          << " dw1=0x" << dw1 << std::dec
		          << " as_lat=" << as_lat
		          << " as_lon=" << as_lon
		          << " dw0_as_alt_cm=" << (dw0 * 0.01)
		          << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FFFB(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFFFB (65531) - Deere proprietary, possibly GNSS position or time
	if (frame.dataLength < 8)
		return;

	std::uint32_t dw0 =
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24);
	std::uint32_t dw1 =
	  static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[6]) << 16) | (static_cast<std::uint32_t>(frame.data[7]) << 24);

	double as_lat = dw0 * 1e-7 - 210.0;
	double as_lon = dw1 * 1e-7 - 210.0;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFFFB candidate: dw0=0x" << std::hex << dw0
		          << " dw1=0x" << dw1 << std::dec
		          << " as_lat=" << as_lat
		          << " as_lon=" << as_lon
		          << " dw0_as_alt_cm=" << (dw0 * 0.01)
		          << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_ACFF(const isobus::CANMessageFrame &frame)
{
	// PGN 0xACFF / 0xAC00 - Deere proprietary transport/multi-packet or GNSS extended
	if (frame.dataLength < 2)
		return;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xACFF candidate: first payload logged at discovery" << std::endl;
		logged = true;
	}
}

void GnssReceiver::parse_candidate_FFFF(const isobus::CANMessageFrame &frame)
{
	// PGN 0xFFFF (65535) - Deere proprietary
	if (frame.dataLength < 2)
		return;

	static bool logged = false;
	if (!logged)
	{
		std::cout << "[GnssReceiver] PGN 0xFFFF candidate: first payload logged at discovery" << std::endl;
		logged = true;
	}
}

// ============================================================================
// PANDA generation
// ============================================================================

void GnssReceiver::send_panda_if_ready(std::shared_ptr<UdpConnections> udp)
{
	if (!isobus::SystemTiming::time_expired_ms(lastPandaSend, 100))
		return;

	// Periodic PGN summary (every 30s)
	if (isobus::SystemTiming::time_expired_ms(lastPgnSummaryLog, 30000))
	{
		std::lock_guard<std::mutex> lock(trackerMutex);
		if (!pgnTrackers.empty())
		{
			std::cout << "[GnssReceiver] === PGN Summary (SA=0x"
			          << std::hex << static_cast<int>(SF3000_SOURCE_ADDRESS) << std::dec << ") ===" << std::endl;
			for (auto &[pgn, t] : pgnTrackers)
			{
				std::uint32_t elapsed = t.lastSeenMs - t.firstSeenMs;
				double hz = (elapsed > 0 && t.count > 1) ? ((t.count - 1) * 1000.0 / elapsed) : 0.0;
				std::cout << "  PGN 0x" << std::hex << std::uppercase << pgn << std::dec
				          << ": count=" << t.count
				          << " ~" << std::fixed << std::setprecision(1) << hz << "Hz"
				          << " last=[";
				for (int i = 0; i < t.lastLength && i < 8; i++)
				{
					if (i > 0)
						std::cout << " ";
					std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(t.lastPayload[i]);
				}
				std::cout << "]" << std::dec << std::endl;
			}
		}
		lastPgnSummaryLog = isobus::SystemTiming::get_timestamp_ms();
	}

	GnssData snapshot;
	{
		std::lock_guard<std::mutex> lock(dataMutex);
		snapshot = data;
	}

	if (!snapshot.has_position)
	{
		if (isobus::SystemTiming::time_expired_ms(lastWaitingLog, 5000))
		{
			std::cout << "[GnssReceiver] Waiting for GNSS position fix..."
			          << " time=" << (snapshot.has_time ? "OK" : "no")
			          << " alt=" << (snapshot.has_altitude ? "OK" : "no")
			          << " hdg=" << (snapshot.has_heading ? "OK" : "no")
			          << " spd=" << (snapshot.has_speed ? "OK" : "no")
			          << std::endl;
			lastWaitingLog = isobus::SystemTiming::get_timestamp_ms();
		}
		return;
	}

	std::string sentence = build_panda(snapshot);

	if (!pandaSending)
	{
		std::cout << "[GnssReceiver] Sending first $PANDA sentence: " << sentence.substr(0, sentence.size() - 2) << std::endl;
		pandaSending = true;
	}

	if (isobus::SystemTiming::time_expired_ms(lastPandaLog, 10000))
	{
		std::cout << "[GnssReceiver] $PANDA: lat=" << snapshot.latitude_deg
		          << " lon=" << snapshot.longitude_deg
		          << " alt=" << snapshot.altitude_m
		          << " hdg=" << snapshot.heading_deg
		          << " spd=" << snapshot.speed_kmh << "km/h"
		          << std::endl;
		lastPandaLog = isobus::SystemTiming::get_timestamp_ms();
	}

	udp->send_raw(sentence);
	lastPandaSend = isobus::SystemTiming::get_timestamp_ms();
}

std::string GnssReceiver::build_panda(const GnssData &snapshot) const
{
	// --- Time: HHMMSS.CC ---
	std::uint8_t ss = (snapshot.minute_ms / 1000) % 60;
	std::uint8_t cc = (snapshot.minute_ms % 1000) / 10;
	char timeStr[12];
	std::snprintf(timeStr, sizeof(timeStr), "%02d%02d%02d.%02d",
	              snapshot.hour, snapshot.minute, ss, cc);

	// --- Latitude: DDMM.MMMM + N/S ---
	double absLat = std::fabs(snapshot.latitude_deg);
	int latDeg = static_cast<int>(absLat);
	double latMin = (absLat - latDeg) * 60.0;
	char latStr[16];
	std::snprintf(latStr, sizeof(latStr), "%02d%07.4f", latDeg, latMin);
	char latHemi = snapshot.latitude_deg >= 0.0 ? 'N' : 'S';

	// --- Longitude: DDDMM.MMMM + E/W ---
	double absLon = std::fabs(snapshot.longitude_deg);
	int lonDeg = static_cast<int>(absLon);
	double lonMin = (absLon - lonDeg) * 60.0;
	char lonStr[16];
	std::snprintf(lonStr, sizeof(lonStr), "%03d%07.4f", lonDeg, lonMin);
	char lonHemi = snapshot.longitude_deg >= 0.0 ? 'E' : 'W';

	// --- Altitude ---
	char altStr[16];
	std::snprintf(altStr, sizeof(altStr), "%.2f", snapshot.has_altitude ? snapshot.altitude_m : 0.0);

	// --- Speed (km/h -> knots) ---
	double speedKnots = snapshot.has_speed ? snapshot.speed_kmh / 1.852 : 0.0;
	char speedStr[16];
	std::snprintf(speedStr, sizeof(speedStr), "%.2f", speedKnots);

	// --- Heading ---
	char headingStr[16];
	std::snprintf(headingStr, sizeof(headingStr), "%.1f", snapshot.has_heading ? snapshot.heading_deg : 0.0);

	// --- Assemble body (everything between $ and *) ---
	char body[256];
	std::snprintf(body, sizeof(body),
	              "PANDA,%s,%s,%c,%s,%c,%d,%d,%.1f,%s,%.1f,%s,%s,%.1f,%.1f,%.1f",
	              timeStr,
	              latStr, latHemi,
	              lonStr, lonHemi,
	              4,   // fixType (RTK default)
	              18,  // satellites default
	              0.8, // HDOP default
	              altStr,
	              0.0, // ageDGPS
	              speedStr,
	              headingStr,
	              0.0, // roll
	              0.0, // pitch
	              0.0  // yawRate
	);

	// --- Checksum: XOR of all chars in body ---
	std::uint8_t cs = 0;
	for (const char *p = body; *p != '\0'; p++)
	{
		cs ^= static_cast<std::uint8_t>(*p);
	}

	char checksum[4];
	std::snprintf(checksum, sizeof(checksum), "%02X", cs);

	// --- Final sentence ---
	std::string sentence = "$";
	sentence += body;
	sentence += "*";
	sentence += checksum;
	sentence += "\r\n";

	return sentence;
}
