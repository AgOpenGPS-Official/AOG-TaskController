/**
 * @brief ISO 11783-7 Tractor Facilities (PGN 65033) implementation.
 *
 * @see ISO 11783-7:2009, B.24.3 (PGN 65033) and B.24.2 (PGN 65032).
 */

#include "tractor_facilities.hpp"

#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_parameter_group_number_request_protocol.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/utility/system_timing.hpp"

#include "logging_utils.hpp"
#include "settings.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>

// ---------------------------------------------------------------------------
// Bit helpers
// ---------------------------------------------------------------------------

/// Set bit @p n (1-based, per ISO convention: bit 1 = LSB) in @p byte.
static inline void set_bit(std::uint8_t &byte, int n)
{
	byte |= static_cast<std::uint8_t>(1u << (n - 1));
}

/// Read bit @p n (1-based) from @p byte.
static inline bool get_bit(std::uint8_t byte, int n)
{
	return (byte >> (n - 1)) & 1u;
}

// ---------------------------------------------------------------------------
// Encode / Decode
// ---------------------------------------------------------------------------

std::array<std::uint8_t, 8> encode_facilities(const Facilities &f)
{
	std::array<std::uint8_t, 8> p{}; // Zero-initialised: all reserved bits = 0.

	// -- Byte 1 ----------------------------------------------------------
	// Bits 8,7 = TECU class.  ISO mapping: 00=Class 1, 01=Class 2,
	// 10=Class 3, 11=Not available.  The Facilities struct stores the
	// class number (1-3) so we subtract 1; anything else becomes 3 (N/A).
	std::uint8_t classBits;
	switch (f.tecuClass)
	{
		case 1:
			classBits = 0;
			break; // 00 = Class 1
		case 2:
			classBits = 1;
			break; // 01 = Class 2
		case 3:
			classBits = 2;
			break; // 10 = Class 3
		default:
			classBits = 3;
			break; // 11 = Not available
	}
	p[0] = static_cast<std::uint8_t>(classBits << 6);
	if (f.powerKeySwitch)
		set_bit(p[0], 6);
	if (f.powerMaxTime)
		set_bit(p[0], 5);
	if (f.powerMaintain)
		set_bit(p[0], 4);
	if (f.wheelBasedSpeed)
		set_bit(p[0], 3);
	if (f.groundBasedSpeed)
		set_bit(p[0], 2);
	if (f.engineSpeed)
		set_bit(p[0], 1);

	// -- Byte 2 ----------------------------------------------------------
	if (f.rearHitchPosition)
		set_bit(p[1], 8);
	if (f.rearHitchInWork)
		set_bit(p[1], 7);
	if (f.rearPtoShaftSpeed)
		set_bit(p[1], 6);
	if (f.rearPtoShaftEngagement)
		set_bit(p[1], 5);
	if (f.minimalLighting)
		set_bit(p[1], 4);
	if (f.languageCommandStorage)
		set_bit(p[1], 3);
	// bits 2,1 reserved → 0

	// -- Byte 3 ----------------------------------------------------------
	if (f.timeDate)
		set_bit(p[2], 8);
	if (f.groundBasedDistance)
		set_bit(p[2], 7);
	if (f.groundBasedDirection)
		set_bit(p[2], 6);
	if (f.wheelBasedDistance)
		set_bit(p[2], 5);
	if (f.wheelBasedDirection)
		set_bit(p[2], 4);
	if (f.rearDraft)
		set_bit(p[2], 3);
	if (f.fullImplementLighting)
		set_bit(p[2], 2);
	if (f.estimatedValveStatus)
		set_bit(p[2], 1);

	// -- Byte 4 ----------------------------------------------------------
	if (f.rearHitchPositionCommand)
		set_bit(p[3], 8);
	if (f.rearPtoSpeedCommand)
		set_bit(p[3], 7);
	if (f.rearPtoEngagementCommand)
		set_bit(p[3], 6);
	if (f.auxiliaryValveCommands)
		set_bit(p[3], 5);
	if (f.limitRequestStatusReporting)
		set_bit(p[3], 4);
	// bits 3-1 reserved → 0

	// -- Byte 5 ----------------------------------------------------------
	if (f.navigationalHighOutputPosition)
		set_bit(p[4], 8);
	if (f.navigationalPositionData)
		set_bit(p[4], 7);
	if (f.navigationalPseudoRangeNoise)
		set_bit(p[4], 6);
	// bit 5 reserved → 0
	if (f.operatorExternalLightControls)
		set_bit(p[4], 4);
	if (f.selectedSpeed)
		set_bit(p[4], 3);
	if (f.selectedSpeedControl)
		set_bit(p[4], 2);
	if (f.directionControl)
		set_bit(p[4], 1);

	// -- Byte 6 ----------------------------------------------------------
	if (f.frontHitchPosition)
		set_bit(p[5], 8);
	if (f.frontHitchInWork)
		set_bit(p[5], 7);
	if (f.frontPtoShaftSpeed)
		set_bit(p[5], 6);
	if (f.frontPtoShaftEngagement)
		set_bit(p[5], 5);
	if (f.frontDraft)
		set_bit(p[5], 4);
	if (f.frontHitchPositionCommand)
		set_bit(p[5], 3);
	if (f.frontPtoSpeedCommand)
		set_bit(p[5], 2);
	if (f.frontPtoEngagementCommand)
		set_bit(p[5], 1);

	// -- Byte 7 ----------------------------------------------------------
	// Entirely reserved → 0

	// -- Byte 8 ----------------------------------------------------------
	// Bits 8-2 reserved → 0.
	// Bit 1 (reserved-bit indicator) → 0, signalling modern 0-fill convention.

	return p;
}

Facilities decode_facilities(const std::array<std::uint8_t, 8> &p)
{
	Facilities f;

	// -- Byte 1 ----------------------------------------------------------
	std::uint8_t classBits = static_cast<std::uint8_t>((p[0] >> 6) & 0x03);
	switch (classBits)
	{
		case 0:
			f.tecuClass = 1;
			break; // 00 = Class 1
		case 1:
			f.tecuClass = 2;
			break; // 01 = Class 2
		case 2:
			f.tecuClass = 3;
			break; // 10 = Class 3
		default:
			f.tecuClass = 0;
			break; // 11 = Not available
	}
	f.powerKeySwitch = get_bit(p[0], 6);
	f.powerMaxTime = get_bit(p[0], 5);
	f.powerMaintain = get_bit(p[0], 4);
	f.wheelBasedSpeed = get_bit(p[0], 3);
	f.groundBasedSpeed = get_bit(p[0], 2);
	f.engineSpeed = get_bit(p[0], 1);

	// -- Byte 2 ----------------------------------------------------------
	f.rearHitchPosition = get_bit(p[1], 8);
	f.rearHitchInWork = get_bit(p[1], 7);
	f.rearPtoShaftSpeed = get_bit(p[1], 6);
	f.rearPtoShaftEngagement = get_bit(p[1], 5);
	f.minimalLighting = get_bit(p[1], 4);
	f.languageCommandStorage = get_bit(p[1], 3);

	// -- Byte 3 ----------------------------------------------------------
	f.timeDate = get_bit(p[2], 8);
	f.groundBasedDistance = get_bit(p[2], 7);
	f.groundBasedDirection = get_bit(p[2], 6);
	f.wheelBasedDistance = get_bit(p[2], 5);
	f.wheelBasedDirection = get_bit(p[2], 4);
	f.rearDraft = get_bit(p[2], 3);
	f.fullImplementLighting = get_bit(p[2], 2);
	f.estimatedValveStatus = get_bit(p[2], 1);

	// -- Byte 4 ----------------------------------------------------------
	f.rearHitchPositionCommand = get_bit(p[3], 8);
	f.rearPtoSpeedCommand = get_bit(p[3], 7);
	f.rearPtoEngagementCommand = get_bit(p[3], 6);
	f.auxiliaryValveCommands = get_bit(p[3], 5);
	f.limitRequestStatusReporting = get_bit(p[3], 4);

	// -- Byte 5 ----------------------------------------------------------
	f.navigationalHighOutputPosition = get_bit(p[4], 8);
	f.navigationalPositionData = get_bit(p[4], 7);
	f.navigationalPseudoRangeNoise = get_bit(p[4], 6);
	f.operatorExternalLightControls = get_bit(p[4], 4);
	f.selectedSpeed = get_bit(p[4], 3);
	f.selectedSpeedControl = get_bit(p[4], 2);
	f.directionControl = get_bit(p[4], 1);

	// -- Byte 6 ----------------------------------------------------------
	f.frontHitchPosition = get_bit(p[5], 8);
	f.frontHitchInWork = get_bit(p[5], 7);
	f.frontPtoShaftSpeed = get_bit(p[5], 6);
	f.frontPtoShaftEngagement = get_bit(p[5], 5);
	f.frontDraft = get_bit(p[5], 4);
	f.frontHitchPositionCommand = get_bit(p[5], 3);
	f.frontPtoSpeedCommand = get_bit(p[5], 2);
	f.frontPtoEngagementCommand = get_bit(p[5], 1);

	return f;
}

// ---------------------------------------------------------------------------
// Time/Date
// ---------------------------------------------------------------------------

int utc_offset_minutes(const std::tm &utc, const std::tm &local)
{
	// The two clocks are at most a day apart. Across New Year tm_yday jumps, so compare the years first.
	int dayDifference = local.tm_yday - utc.tm_yday;
	if (local.tm_year != utc.tm_year)
	{
		dayDifference = (local.tm_year > utc.tm_year) ? 1 : -1;
	}
	return (dayDifference * 24 * 60) + ((local.tm_hour - utc.tm_hour) * 60) + (local.tm_min - utc.tm_min);
}

std::array<std::uint8_t, 8> encode_time_date(const std::tm &utc, const std::tm &local, int milliseconds)
{
	const int offsetMinutes = utc_offset_minutes(utc, local);
	const int seconds = std::min(utc.tm_sec, 59); // tm_sec is 60 during a leap second
	const int quarterOfDay = utc.tm_hour / 6;

	std::array<std::uint8_t, 8> p{};
	p[0] = static_cast<std::uint8_t>((seconds * 4) + (milliseconds / 250)); // 0.25 s/bit
	p[1] = static_cast<std::uint8_t>(utc.tm_min);
	p[2] = static_cast<std::uint8_t>(utc.tm_hour);
	p[3] = static_cast<std::uint8_t>(utc.tm_mon + 1);
	p[4] = static_cast<std::uint8_t>(((utc.tm_mday - 1) * 4) + quarterOfDay + 1); // 0.25 day/bit, 0 = not a day
	p[5] = static_cast<std::uint8_t>(utc.tm_year + 1900 - 1985);
	// Both offsets carry the sign: -3:30 is -3 h and -30 min
	p[6] = static_cast<std::uint8_t>((offsetMinutes % 60) + 125);
	p[7] = static_cast<std::uint8_t>((offsetMinutes / 60) + 125);
	return p;
}

/// Read the PC clock. False if it is outside what the message can carry, e.g. a clock that was never set.
static bool read_system_time_date(std::array<std::uint8_t, 8> &payload)
{
	const auto now = std::chrono::system_clock::now();
	const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
	const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

	std::tm utc{};
	std::tm local{};
#if defined(_WIN32)
	const bool converted = (0 == gmtime_s(&utc, &seconds)) && (0 == localtime_s(&local, &seconds));
#else
	const bool converted = (nullptr != gmtime_r(&seconds, &utc)) && (nullptr != localtime_r(&seconds, &local));
#endif
	const int year = utc.tm_year + 1900;
	if (!converted || (year < 1985) || (year > 2235))
	{
		return false;
	}
	payload = encode_time_date(utc, local, static_cast<int>(milliseconds));
	return true;
}

// ---------------------------------------------------------------------------
// TractorFacilities
// ---------------------------------------------------------------------------

TractorFacilities::TractorFacilities(
  std::shared_ptr<isobus::InternalControlFunction> tecuCF,
  std::shared_ptr<Settings> settings) :
  tecuCF(std::move(tecuCF)),
  settings(std::move(settings))
{
}

TractorFacilities::~TractorFacilities()
{
	if (!initialized)
	{
		return;
	}
	// The request protocol calls on_pgn_request() under its own lock, so after this returns it no longer
	// runs. The stack may still call on_required_facilities() once after removal (it iterates a copy of
	// the global callbacks), which is harmless: that callback only logs and never touches this object.
	if (auto pgnRequestProtocol = tecuCF ? tecuCF->get_pgn_request_protocol().lock() : nullptr)
	{
		pgnRequestProtocol->remove_pgn_request_callback(PGN_TRACTOR_FACILITIES, &TractorFacilities::on_pgn_request, this);
	}
	isobus::CANNetworkManager::CANNetwork.remove_global_parameter_group_number_callback(
	  PGN_REQUIRED_TRACTOR_FACILITIES, &TractorFacilities::on_required_facilities, this);
	if (auto pgnRequestProtocol = tecuCF ? tecuCF->get_pgn_request_protocol().lock() : nullptr)
	{
		pgnRequestProtocol->remove_pgn_request_callback(PGN_TIME_DATE, &TractorFacilities::on_time_date_request, this);
	}
}

void TractorFacilities::set_provided_facilities(const Facilities &facilities)
{
	std::lock_guard<std::mutex> lock(providedFacilitiesMutex);
	providedFacilities = facilities;
	hasProvidedFacilities = true;
}

void TractorFacilities::set_speed_messages_interface(isobus::SpeedMessagesInterface *iface)
{
	speedMessagesInterface = iface;
}

void TractorFacilities::set_nmea2000_message_interface(isobus::NMEA2000MessageInterface *iface)
{
	nmea2000MessageInterface = iface;
}

void TractorFacilities::set_time_date_active(bool active)
{
	timeDateActive = active;
}

std::array<std::uint8_t, 8> TractorFacilities::build_payload() const
{
	{
		std::lock_guard<std::mutex> lock(providedFacilitiesMutex);
		if (hasProvidedFacilities)
		{
			Facilities provided = providedFacilities;
			provided.timeDate = timeDateActive;
			return encode_facilities(provided);
		}
	}

	Facilities f;
	f.tecuClass = 1; // Class 1 – the only class we claim.

	// Ground-based speed (PGN 65097 / 0xFE49) is always broadcast when
	// the SpeedMessagesInterface exists.
	const bool groundSpeedActive = (speedMessagesInterface != nullptr);
	f.groundBasedSpeed = groundSpeedActive;
	f.groundBasedDistance = groundSpeedActive;
	f.groundBasedDirection = groundSpeedActive;

	// Wheel-based speed (PGN 65096 / 0xFE48) is also always broadcast
	// when the interface exists (constructor parameter is `true`).
	const bool wheelSpeedActive = (speedMessagesInterface != nullptr);
	f.wheelBasedSpeed = wheelSpeedActive;
	f.wheelBasedDistance = wheelSpeedActive;
	f.wheelBasedDirection = wheelSpeedActive;

	// Time/date (PGN 65254 / FEE6) – set by the application layer while
	// the TECU answers requests for it.
	f.timeDate = timeDateActive;

	// Everything else stays 0: we have no engine data, hitch feedback,
	// PTO, auxiliary valves, lighting, language storage (PGN 65039),
	// selected speed (PGN 65265), or NMEA 2000
	// position forwarding over Fast Packet.

	return encode_facilities(f);
}

bool TractorFacilities::initialize()
{
	if (!tecuCF || !tecuCF->get_address_valid())
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] TECU not available, skipping initialization." << std::endl;
		return false;
	}

	// Register PGN 65033 request callback on the TECU's ICF.
	auto pgnRequestProtocol = tecuCF->get_pgn_request_protocol().lock();
	if (!pgnRequestProtocol)
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] PGN request protocol not available for TECU." << std::endl;
		return false;
	}

	bool registered = pgnRequestProtocol->register_pgn_request_callback(
	  PGN_TRACTOR_FACILITIES, &TractorFacilities::on_pgn_request, this);
	if (!registered)
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] Failed to register PGN 65033 request callback." << std::endl;
		return false;
	}
	std::cout << "[" << get_timestamp() << "] [TractorFacilities] Registered PGN 65033 request callback on TECU (SA "
	          << static_cast<int>(tecuCF->get_address()) << ")." << std::endl;

	// Register a global receive handler for PGN 65032 (Required Tractor
	// Facilities) – diagnostic logging only.
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(
	  PGN_REQUIRED_TRACTOR_FACILITIES, &TractorFacilities::on_required_facilities, this);
	std::cout << "[" << get_timestamp() << "] [TractorFacilities] Registered PGN 65032 diagnostic listener." << std::endl;

	if (!pgnRequestProtocol->register_pgn_request_callback(PGN_TIME_DATE, &TractorFacilities::on_time_date_request, this))
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] Failed to register PGN 65254 request callback." << std::endl;
	}

	initialized = true;
	return true;
}

bool TractorFacilities::send_facilities_response(bool isPowerUp)
{
	if (!tecuCF || !tecuCF->get_address_valid())
	{
		return false;
	}

	auto payload = build_payload();
	bool sent = isobus::CANNetworkManager::CANNetwork.send_can_message(
	  PGN_TRACTOR_FACILITIES, payload.data(), payload.size(), tecuCF);

	if (sent)
	{
		std::ostringstream hex;
		hex << std::hex;
		for (std::size_t i = 0; i < payload.size(); ++i)
		{
			if (i != 0)
				hex << ' ';
			hex << "0x" << static_cast<int>(payload[i]);
		}
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] Sent PGN 65033 ("
		          << (isPowerUp ? "power-up" : "requested") << "): [" << hex.str() << "]" << std::endl;
	}
	else
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] Failed to send PGN 65033." << std::endl;
	}
	return sent;
}

// ---------------------------------------------------------------------------
// Static callbacks
// ---------------------------------------------------------------------------

bool TractorFacilities::on_time_date_request(
  std::uint32_t parameterGroupNumber,
  std::shared_ptr<isobus::ControlFunction> /*requestingControlFunction*/,
  bool &acknowledge,
  isobus::AcknowledgementType & /*acknowledgeType*/,
  void *parentPointer)
{
	auto *self = static_cast<TractorFacilities *>(parentPointer);
	std::array<std::uint8_t, 8> payload{};
	// Returning false leaves it to the Tractor ECU that does provide it: the stack ignores a global
	// request and NACKs one addressed to us
	if ((PGN_TIME_DATE != parameterGroupNumber) || (nullptr == self) || !self->timeDateActive || !read_system_time_date(payload))
	{
		return false;
	}

	acknowledge = false; // Sending the requested PGN is the response
	const bool sent = isobus::CANNetworkManager::CANNetwork.send_can_message(PGN_TIME_DATE, payload.data(), payload.size(), self->tecuCF);
	if (sent && !self->timeDateRequestLogged.exchange(true))
	{
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] Answering requests for PGN 65254 (Time/Date) from the PC clock." << std::endl;
	}
	return sent;
}

bool TractorFacilities::on_pgn_request(
  std::uint32_t parameterGroupNumber,
  std::shared_ptr<isobus::ControlFunction> requestingControlFunction,
  bool &acknowledge,
  isobus::AcknowledgementType & /*acknowledgeType*/,
  void *parentPointer)
{
	if (parameterGroupNumber != PGN_TRACTOR_FACILITIES || !parentPointer || !requestingControlFunction)
	{
		return false;
	}

	auto *self = static_cast<TractorFacilities *>(parentPointer);

	// Sending the requested PGN *is* the response; an ACK on top would be
	// redundant and confusing to some implement stacks.
	acknowledge = false;

	// Log once per requester source address at info level.
	std::uint8_t sa = requestingControlFunction->get_address();
	if (self->loggedRequesters.find(sa) == self->loggedRequesters.end())
	{
		self->loggedRequesters.insert(sa);

		auto payload = self->build_payload();
		std::ostringstream hex;
		hex << std::hex;
		for (std::size_t i = 0; i < payload.size(); ++i)
		{
			if (i != 0)
				hex << ' ';
			hex << "0x" << static_cast<int>(payload[i]);
		}
		std::cout << "[" << get_timestamp() << "] [TractorFacilities] PGN 65033 requested by SA "
		          << static_cast<int>(sa) << ", advertising facilities: [" << hex.str() << "]" << std::endl;
	}

	return self->send_facilities_response(false);
}

void TractorFacilities::on_required_facilities(
  const isobus::CANMessage &message,
  void *parentPointer)
{
	if (!parentPointer)
	{
		return;
	}

	const auto &data = message.get_data();
	if (data.size() < 8)
	{
		return;
	}

	auto sourceCF = message.get_source_control_function();
	std::uint8_t sa = sourceCF ? sourceCF->get_address() : 0xFF;

	// Decode and log — diagnostic only, do not change our
	// response based on what the implement asks for.
	std::array<std::uint8_t, 8> raw{};
	for (std::size_t i = 0; i < 8 && i < data.size(); ++i)
	{
		raw[i] = data[i];
	}

	// A simple hex dump avoids pulling in the full Facilities decode for a
	// diagnostic message.
	std::ostringstream hex;
	hex << std::hex;
	for (std::size_t i = 0; i < raw.size(); ++i)
	{
		if (i != 0)
			hex << ' ';
		hex << "0x" << static_cast<int>(raw[i]);
	}
	isobus::CANStackLogger::debug("[TractorFacilities] PGN 65032 from SA " + std::to_string(static_cast<int>(sa)) +
	                              ": required facilities [" + hex.str() + "]");
}
