/**
 * @brief Sharing the implement bus with other Tractor ECUs (ISO 11783-9:2012, 4.4.2.1).
 *
 * When several Tractor ECUs are on the implement bus, the one with function
 * instance 0 is the primary. A Tractor ECU with a higher function instance
 * "shall not provide any messages which are provided by a Tractor ECU with a
 * lower function instance", requests the tractor facility response (PGN 65033)
 * from those, and "shall only set those facilities bit(s) that are not available
 * in the other Tractor ECUs".
 *
 * Our TECU runs as the primary when it is alone, and as a secondary otherwise.
 * This file holds the decision of what a secondary sends (pure functions, unit
 * tested) and OtherTecuMonitor, which collects what the other TECUs advertise
 * and actually send.
 */

#pragma once

#include "tractor_facilities.hpp"

#include "isobus/isobus/can_control_function.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/can_message_frame.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

/// Messages our TECU can put on the bus
enum class TecuMessage : std::uint8_t
{
	GroundBasedSpeed, ///< PGN 65097, ground-based speed, distance and direction
	WheelBasedSpeed, ///< PGN 65096, wheel-based speed, distance and direction
	MachineSelectedSpeed, ///< PGN 61474
	VehicleDirectionSpeed, ///< PGN 65256, J1939 vehicle direction/speed (no facility bit)
	CogSogRapidUpdate, ///< PGN 129026, NMEA 2000 COG & SOG rapid update (no facility bit)
	TimeDate, ///< PGN 65254
	Count
};

constexpr std::size_t TECU_MESSAGE_COUNT = static_cast<std::size_t>(TecuMessage::Count);

/// @brief The PGN a TecuMessage is sent with
std::uint32_t tecu_message_pgn(TecuMessage message);

/// @brief Short name for logs, e.g. "ground-based speed (PGN 65097)"
std::string tecu_message_name(TecuMessage message);

/// Which of our messages to send
struct TecuMessagePlan
{
	std::array<bool, TECU_MESSAGE_COUNT> send{};

	bool sends(TecuMessage message) const
	{
		return send[static_cast<std::size_t>(message)];
	}
	bool operator==(const TecuMessagePlan &other) const
	{
		return send == other.send;
	}
	bool operator!=(const TecuMessagePlan &other) const
	{
		return send != other.send;
	}
};

/// @brief Decide which messages our TECU sends.
/// @param isPrimary True when our TECU is the primary (function instance 0): it sends everything.
/// @param advertisedByOthers Facilities the other (lower instance) TECUs report in PGN 65033, combined.
/// @param sentByOthers Messages seen recently from another TECU, whether advertised or not.
/// @returns As a secondary, only the messages no other TECU provides (ISO 11783-9 4.4.2.1 c).
TecuMessagePlan plan_tecu_messages(bool isPrimary,
                                   const Facilities &advertisedByOthers,
                                   const std::array<bool, TECU_MESSAGE_COUNT> &sentByOthers);

/// @brief The facilities to advertise for a plan (ISO 11783-9 4.4.2.1 d): exactly the messages we send.
/// The time/date bit is left clear; it is set separately while time/date is actually provided.
Facilities facilities_for_plan(const TecuMessagePlan &plan);

/// @brief Bitwise OR of two facility sets (what any of several TECUs provides)
Facilities merge_facilities(const Facilities &a, const Facilities &b);

/// @brief The J1939 PGN of a 29-bit CAN identifier (PDU1 destination address removed)
std::uint32_t pgn_from_can_identifier(std::uint32_t identifier);

/// Another Tractor ECU on the bus
struct OtherTecu
{
	std::shared_ptr<isobus::ControlFunction> controlFunction;
	std::uint8_t address = 0xFE;
	std::uint8_t functionInstance = 0;
	std::uint16_t manufacturerCode = 0;
};

/// @brief The other Tractor ECUs that have sent Time/Date (PGN 65254) since they appeared on the bus.
///
/// Time/Date is only sent on request (ISO 11783-7 B.1), so a time window cannot tell whether a lower Tractor
/// ECU provides it: with requests further apart than the window, both TECUs would answer every one. Instead a
/// TECU counts as providing Time/Date from the first time it sends it until it leaves the bus (forget()).
/// mark_sent() runs on the CAN stack's background thread, forget() and any() on the main thread.
class TimeDateProviders
{
public:
	void mark_sent(std::uint8_t address)
	{
		sent[address] = true;
	}
	void forget(std::uint8_t address)
	{
		sent[address] = false;
	}
	bool any() const
	{
		for (const auto &address : sent)
		{
			if (address)
			{
				return true;
			}
		}
		return false;
	}

private:
	std::array<std::atomic<bool>, 256> sent{};
};

/// @brief Tracks the other Tractor ECUs: who they are, the facilities they advertise and what they send.
///
/// refresh() and the queries run on the main thread. on_frame() and the PGN 65033 callback run on the
/// CAN stack's background thread (see docs/CONCURRENCY.md), so the shared state is atomic or under a mutex.
/// The object lives as long as the Application: the stack may still call a global PGN callback once
/// after it was removed, so this must never be destroyed while the stack runs.
class OtherTecuMonitor
{
public:
	/// How long a message seen from another TECU counts as "sent by it" (not time/date, see TimeDateProviders)
	static constexpr std::uint32_t SENT_BY_OTHERS_WINDOW_MS = 3000;

	/// @brief Register the PGN 65033 listener. Call once.
	void initialize();

	/// @brief Re-read the other TECUs from the network manager, excluding our own control functions.
	/// Only TECUs with a function instance up to maxFunctionInstance count: a secondary defers to lower
	/// instances only (ISO 11783-9 4.4.2.1 c), so two secondaries never wait on each other. Forgets
	/// facilities of TECUs that have gone.
	/// @returns Those TECUs currently on the bus, lowest function instance first.
	std::vector<OtherTecu> refresh(std::uint8_t maxFunctionInstance);

	/// @brief Called for every received CAN frame (background thread)
	void on_frame(const isobus::CANMessageFrame &frame);

	/// @brief The facilities advertised by the other TECUs, combined
	Facilities advertised_facilities() const;

	/// @brief Whether every other TECU has answered with PGN 65033
	bool all_facilities_received() const;

	/// @brief Messages seen from another TECU within SENT_BY_OTHERS_WINDOW_MS. Time/date: sent by a TECU still on the bus
	std::array<bool, TECU_MESSAGE_COUNT> sent_by_others() const;

	/// @brief Whether we already asked this address for PGN 65033 since it appeared
	bool facilities_requested(std::uint8_t address) const;
	void mark_facilities_requested(std::uint8_t address);

private:
	static void on_facilities_response(const isobus::CANMessage &message, void *parent);

	std::array<std::atomic<bool>, 256> isOtherTecuAddress{};
	std::array<std::atomic<std::uint32_t>, TECU_MESSAGE_COUNT> lastSentByOthersMs{}; ///< 0 = never
	TimeDateProviders timeDateProviders;

	mutable std::mutex mutex; ///< Guards the members below
	std::map<std::uint8_t, Facilities> facilitiesByAddress;
	std::set<std::uint8_t> otherAddresses;
	std::set<std::uint8_t> requestedAddresses;
	std::map<std::uint8_t, std::uint64_t> nameByAddress; ///< To notice a different TECU taking over an address
};
