/**
 * @author Qoder
 * @brief GNSS receiver that parses J1939 PGNs from a John Deere SF3000 and generates $PANDA NMEA sentences
 * @version 0.1
 * @date 2025-03-14
 */

#include "gnss_receiver.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

#include "isobus/utility/system_timing.hpp"

GnssReceiver::GnssReceiver()
{
}

GnssReceiver::~GnssReceiver()
{
	isobus::CANHardwareInterface::get_can_frame_received_event_dispatcher()
	  .remove_listener(canFrameHandle);
}

void GnssReceiver::initialize()
{
	canFrameHandle = isobus::CANHardwareInterface::get_can_frame_received_event_dispatcher()
	                   .add_listener([this](const isobus::CANMessageFrame &frame) {
		                   on_can_frame(frame);
	                   });
	std::cout << "[GnssReceiver] CAN frame listener registered for SF3000 (SA=0x"
	          << std::hex << static_cast<int>(SF3000_SOURCE_ADDRESS) << std::dec << ")" << std::endl;
}

void GnssReceiver::on_can_frame(const isobus::CANMessageFrame &frame)
{
	if (!frame.isExtendedFrame)
		return;

	std::uint8_t sa = frame.identifier & 0xFF;
	std::uint8_t pf = (frame.identifier >> 16) & 0xFF;

	// Extract PGN from 29-bit CAN identifier
	std::uint32_t pgn;
	if (pf >= 0xF0)
	{
		// PDU2 format: PS byte is group extension, part of PGN
		pgn = (frame.identifier >> 8) & 0x3FFFF;
	}
	else
	{
		// PDU1 format: PS byte is destination address, not part of PGN
		pgn = (frame.identifier >> 8) & 0x3FF00;
	}

	// SF3000-specific PGNs (gate on source address)
	if (sa == SF3000_SOURCE_ADDRESS)
	{
		switch (pgn)
		{
			case 0xFEF3:
				parse_position(frame);
				break;
			case 0xFEF0:
				parse_time_date(frame);
				break;
			case 0xFEF2:
				parse_altitude(frame);
				break;
			case 0xFE45:
				parse_heading(frame);
				break;
			default:
				break;
		}
	}

	// Speed PGN from any source address
	if (pgn == 0xFEF1)
	{
		parse_speed(frame);
	}
}

void GnssReceiver::parse_position(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 8)
		return;

	auto raw_lat = static_cast<std::int32_t>(
	  static_cast<std::uint32_t>(frame.data[0]) | (static_cast<std::uint32_t>(frame.data[1]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[2]) << 16) | (static_cast<std::uint32_t>(frame.data[3]) << 24));
	auto raw_lon = static_cast<std::int32_t>(
	  static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8) |
	  (static_cast<std::uint32_t>(frame.data[6]) << 16) | (static_cast<std::uint32_t>(frame.data[7]) << 24));

	// 0x7FFFFFFF = "not available" in J1939
	if (raw_lat == 0x7FFFFFFF || raw_lon == 0x7FFFFFFF)
		return;

	double lat = raw_lat * 1e-7;
	double lon = raw_lon * 1e-7;

	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
	{
		std::cout << "[GnssReceiver] PGN 0xFEF3 position out of range: lat=" << lat << " lon=" << lon << std::endl;
		return;
	}

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_position)
	{
		std::cout << "[GnssReceiver] First position fix received: lat=" << lat << " lon=" << lon << std::endl;
	}
	data.latitude_deg = lat;
	data.longitude_deg = lon;
	data.has_position = true;
}

void GnssReceiver::parse_time_date(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 4)
		return;

	std::uint16_t ms_of_minute = static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));
	std::uint8_t min = frame.data[2];
	std::uint8_t hr = frame.data[3];

	if (hr > 23 || min > 59 || ms_of_minute > 59999)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_time)
	{
		std::cout << "[GnssReceiver] First time/date received: "
		          << static_cast<int>(hr) << ":" << static_cast<int>(min) << " (" << ms_of_minute << "ms)" << std::endl;
	}
	data.hour = hr;
	data.minute = min;
	data.minute_ms = ms_of_minute;
	data.has_time = true;
}

void GnssReceiver::parse_altitude(const isobus::CANMessageFrame &frame)
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
		std::cout << "[GnssReceiver] First altitude received: " << (raw_alt * 0.01) << " m" << std::endl;
	}
	data.altitude_m = raw_alt * 0.01;
	data.has_altitude = true;
}

void GnssReceiver::parse_heading(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 2)
		return;

	std::uint16_t raw_heading = static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8));

	if (raw_heading == 0xFFFF)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_heading)
	{
		std::cout << "[GnssReceiver] First heading received: " << (raw_heading * 0.0078125) << " deg" << std::endl;
	}
	data.heading_deg = raw_heading * 0.0078125; // 1/128 degree per bit
	data.has_heading = true;
}

void GnssReceiver::parse_speed(const isobus::CANMessageFrame &frame)
{
	if (frame.dataLength < 3)
		return;

	// Bytes 1-2 (0-indexed) contain wheel-based vehicle speed
	std::uint16_t raw_speed = static_cast<std::uint16_t>(frame.data[1] | (frame.data[2] << 8));

	if (raw_speed == 0xFFFF)
		return;

	std::lock_guard<std::mutex> lock(dataMutex);
	if (!data.has_speed)
	{
		std::cout << "[GnssReceiver] First speed received: " << (raw_speed / 256.0) << " km/h (SA=0x"
		          << std::hex << static_cast<int>(frame.identifier & 0xFF) << std::dec << ")" << std::endl;
	}
	data.speed_kmh = raw_speed / 256.0;
	data.has_speed = true;
}

void GnssReceiver::send_panda_if_ready(std::shared_ptr<UdpConnections> udp)
{
	if (!isobus::SystemTiming::time_expired_ms(lastPandaSend, 100))
		return;

	GnssData snapshot;
	{
		std::lock_guard<std::mutex> lock(dataMutex);
		snapshot = data;
	}

	if (!snapshot.has_position || !snapshot.has_time)
	{
		if (isobus::SystemTiming::time_expired_ms(lastWaitingLog, 5000))
		{
			std::cout << "[GnssReceiver] Waiting for GNSS data... position="
			          << (snapshot.has_position ? "OK" : "MISSING")
			          << " time=" << (snapshot.has_time ? "OK" : "MISSING")
			          << " altitude=" << (snapshot.has_altitude ? "OK" : "no")
			          << " heading=" << (snapshot.has_heading ? "OK" : "no")
			          << " speed=" << (snapshot.has_speed ? "OK" : "no")
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
	// $PANDA,time,lat,N,lon,E,fix,sats,hdop,alt,ageDGPS,speedKnots,heading,roll,pitch,yawRate*CS
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
