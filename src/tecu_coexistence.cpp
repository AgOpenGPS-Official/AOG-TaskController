/**
 * @brief Sharing the implement bus with other Tractor ECUs (ISO 11783-9:2012, 4.4.2.1).
 */

#include "tecu_coexistence.hpp"

#include "isobus/isobus/can_NAME.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/utility/system_timing.hpp"

#include <algorithm>

namespace
{
	constexpr std::size_t index_of(TecuMessage message)
	{
		return static_cast<std::size_t>(message);
	}
}

std::uint32_t tecu_message_pgn(TecuMessage message)
{
	switch (message)
	{
		case TecuMessage::GroundBasedSpeed:
			return 65097;
		case TecuMessage::WheelBasedSpeed:
			return 65096;
		case TecuMessage::MachineSelectedSpeed:
			return 61474;
		case TecuMessage::VehicleDirectionSpeed:
			return 65256;
		case TecuMessage::CogSogRapidUpdate:
			return 129026;
		case TecuMessage::TimeDate:
			return 65254;
		default:
			return 0;
	}
}

std::string tecu_message_name(TecuMessage message)
{
	std::string name;
	switch (message)
	{
		case TecuMessage::GroundBasedSpeed:
			name = "ground-based speed";
			break;
		case TecuMessage::WheelBasedSpeed:
			name = "wheel-based speed";
			break;
		case TecuMessage::MachineSelectedSpeed:
			name = "machine selected speed";
			break;
		case TecuMessage::VehicleDirectionSpeed:
			name = "vehicle direction/speed";
			break;
		case TecuMessage::CogSogRapidUpdate:
			name = "NMEA 2000 COG/SOG";
			break;
		case TecuMessage::TimeDate:
			name = "time/date";
			break;
		default:
			name = "unknown";
			break;
	}
	return name + " (PGN " + std::to_string(tecu_message_pgn(message)) + ")";
}

TecuMessagePlan plan_tecu_messages(bool isPrimary,
                                   const Facilities &advertisedByOthers,
                                   const std::array<bool, TECU_MESSAGE_COUNT> &sentByOthers)
{
	TecuMessagePlan plan;
	if (isPrimary)
	{
		plan.send.fill(true);
		return plan;
	}

	std::array<bool, TECU_MESSAGE_COUNT> advertised{};
	advertised[index_of(TecuMessage::GroundBasedSpeed)] = advertisedByOthers.groundBasedSpeed;
	advertised[index_of(TecuMessage::WheelBasedSpeed)] = advertisedByOthers.wheelBasedSpeed;
	advertised[index_of(TecuMessage::MachineSelectedSpeed)] = advertisedByOthers.selectedSpeed;
	advertised[index_of(TecuMessage::TimeDate)] = advertisedByOthers.timeDate;
	// Vehicle direction/speed and COG/SOG have no facility bit: only what is seen on the bus counts

	for (std::size_t i = 0; i < TECU_MESSAGE_COUNT; i++)
	{
		plan.send[i] = !(advertised[i] || sentByOthers[i]);
	}
	return plan;
}

Facilities facilities_for_plan(const TecuMessagePlan &plan)
{
	Facilities f;
	f.tecuClass = 1;

	const bool ground = plan.sends(TecuMessage::GroundBasedSpeed);
	f.groundBasedSpeed = ground;
	f.groundBasedDistance = ground;
	f.groundBasedDirection = ground;

	const bool wheel = plan.sends(TecuMessage::WheelBasedSpeed);
	f.wheelBasedSpeed = wheel;
	f.wheelBasedDistance = wheel;
	f.wheelBasedDirection = wheel;

	f.selectedSpeed = plan.sends(TecuMessage::MachineSelectedSpeed);
	return f;
}

Facilities merge_facilities(const Facilities &a, const Facilities &b)
{
	// The encoded payload is the bitwise form of every facility, so OR it byte by byte
	const auto pa = encode_facilities(a);
	const auto pb = encode_facilities(b);
	std::array<std::uint8_t, 8> merged{};
	for (std::size_t i = 0; i < merged.size(); i++)
	{
		merged[i] = static_cast<std::uint8_t>(pa[i] | pb[i]);
	}
	Facilities result = decode_facilities(merged);
	// The class field is not a bit set: keep the higher class ("not available" is 0 here)
	result.tecuClass = std::max(a.tecuClass, b.tecuClass);
	return result;
}

std::uint32_t pgn_from_can_identifier(std::uint32_t identifier)
{
	const std::uint32_t pduFormat = (identifier >> 16) & 0xFF;
	const std::uint32_t pduSpecific = (identifier >> 8) & 0xFF;
	const std::uint32_t dataPages = (identifier >> 24) & 0x03; // Extended data page and data page
	// PDU1 (PF < 240): the PS byte is a destination address, not part of the PGN
	return (dataPages << 16) | (pduFormat << 8) | ((pduFormat >= 0xF0) ? pduSpecific : 0);
}

void OtherTecuMonitor::initialize()
{
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(
	  TractorFacilities::PGN_TRACTOR_FACILITIES, &OtherTecuMonitor::on_facilities_response, this);
}

std::vector<OtherTecu> OtherTecuMonitor::refresh(std::uint8_t maxFunctionInstance)
{
	std::vector<OtherTecu> others;
	for (const auto &cf : isobus::CANNetworkManager::CANNetwork.get_control_functions(false))
	{
		if ((nullptr == cf) || !cf->get_address_valid() ||
		    (isobus::ControlFunction::Type::Internal == cf->get_type()) ||
		    (static_cast<std::uint8_t>(isobus::NAME::Function::TractorECU) != cf->get_NAME().get_function_code()) ||
		    (cf->get_NAME().get_function_instance() > maxFunctionInstance))
		{
			continue;
		}
		OtherTecu other;
		other.controlFunction = cf;
		other.address = cf->get_address();
		other.functionInstance = cf->get_NAME().get_function_instance();
		other.manufacturerCode = cf->get_NAME().get_manufacturer_code();
		others.push_back(other);
	}
	std::sort(others.begin(), others.end(), [](const OtherTecu &a, const OtherTecu &b) {
		return a.functionInstance < b.functionInstance;
	});

	std::set<std::uint8_t> addresses;
	std::map<std::uint8_t, std::uint64_t> names;
	for (const auto &other : others)
	{
		addresses.insert(other.address);
		names[other.address] = other.controlFunction->get_NAME().get_full_name();
	}

	std::lock_guard<std::mutex> lock(mutex);
	for (std::size_t address = 0; address < isOtherTecuAddress.size(); address++)
	{
		isOtherTecuAddress[address] = (addresses.count(static_cast<std::uint8_t>(address)) > 0);
	}
	for (auto it = facilitiesByAddress.begin(); it != facilitiesByAddress.end();)
	{
		// A TECU that left, or a different one now at that address, has to answer again
		const auto name = names.find(it->first);
		const auto previous = nameByAddress.find(it->first);
		const bool same = (names.end() != name) && (nameByAddress.end() != previous) && (name->second == previous->second);
		it = same ? std::next(it) : facilitiesByAddress.erase(it);
	}
	for (auto it = requestedAddresses.begin(); it != requestedAddresses.end();)
	{
		const auto name = names.find(*it);
		const auto previous = nameByAddress.find(*it);
		const bool same = (names.end() != name) && (nameByAddress.end() != previous) && (name->second == previous->second);
		it = same ? std::next(it) : requestedAddresses.erase(it);
	}
	otherAddresses = addresses;
	nameByAddress = names;
	return others;
}

void OtherTecuMonitor::on_frame(const isobus::CANMessageFrame &frame)
{
	if (!frame.isExtendedFrame || !isOtherTecuAddress[frame.identifier & 0xFF])
	{
		return;
	}
	const std::uint32_t pgn = pgn_from_can_identifier(frame.identifier);
	for (std::size_t i = 0; i < TECU_MESSAGE_COUNT; i++)
	{
		if (tecu_message_pgn(static_cast<TecuMessage>(i)) == pgn)
		{
			const std::uint32_t now = isobus::SystemTiming::get_timestamp_ms();
			lastSentByOthersMs[i] = (0 == now) ? 1 : now; // 0 means "never"
		}
	}
}

void OtherTecuMonitor::on_facilities_response(const isobus::CANMessage &message, void *parent)
{
	auto *self = static_cast<OtherTecuMonitor *>(parent);
	const auto source = message.get_source_control_function();
	const auto &data = message.get_data();
	if ((nullptr == self) || (nullptr == source) || (data.size() < 8))
	{
		return;
	}
	const std::uint8_t address = source->get_address();
	if (!self->isOtherTecuAddress[address])
	{
		return; // Our own response, or one from something that is not a TECU
	}

	std::array<std::uint8_t, 8> payload{};
	std::copy(data.begin(), data.begin() + 8, payload.begin());
	if (payload[7] & 0x01)
	{
		// Reserved bit indicator set (ISO 11783-7 A.27.6): the newer facility bits are 1-filled, not real
		payload[1] &= 0xFC;
		payload[3] &= 0xF0;
		payload[4] &= 0xE0;
		payload[6] = 0;
		payload[7] = 0;
	}

	std::lock_guard<std::mutex> lock(self->mutex);
	self->facilitiesByAddress[address] = decode_facilities(payload);
}

Facilities OtherTecuMonitor::advertised_facilities() const
{
	std::lock_guard<std::mutex> lock(mutex);
	Facilities combined;
	combined.tecuClass = 0;
	for (const auto &entry : facilitiesByAddress)
	{
		combined = merge_facilities(combined, entry.second);
	}
	return combined;
}

bool OtherTecuMonitor::all_facilities_received() const
{
	std::lock_guard<std::mutex> lock(mutex);
	return std::all_of(otherAddresses.begin(), otherAddresses.end(), [this](std::uint8_t address) {
		return facilitiesByAddress.count(address) > 0;
	});
}

std::array<bool, TECU_MESSAGE_COUNT> OtherTecuMonitor::sent_by_others() const
{
	std::array<bool, TECU_MESSAGE_COUNT> sent{};
	for (std::size_t i = 0; i < TECU_MESSAGE_COUNT; i++)
	{
		const std::uint32_t last = lastSentByOthersMs[i];
		sent[i] = (0 != last) && !isobus::SystemTiming::time_expired_ms(last, SENT_BY_OTHERS_WINDOW_MS);
	}
	return sent;
}

bool OtherTecuMonitor::facilities_requested(std::uint8_t address) const
{
	std::lock_guard<std::mutex> lock(mutex);
	return requestedAddresses.count(address) > 0;
}

void OtherTecuMonitor::mark_facilities_requested(std::uint8_t address)
{
	std::lock_guard<std::mutex> lock(mutex);
	requestedAddresses.insert(address);
}
