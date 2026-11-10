/**
 * @brief Unit tests for sharing the bus with other Tractor ECUs (ISO 11783-9:2012, 4.4.2.1):
 *        which messages a secondary TECU sends, what it advertises, and PGN decoding.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "tecu_coexistence.hpp"

#include <array>
#include <cstdio>

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

static std::array<bool, TECU_MESSAGE_COUNT> nothing_sent()
{
	return {};
}

static void test_primary_sends_everything()
{
	std::printf("test_primary_sends_everything\n");
	Facilities others;
	others.groundBasedSpeed = true;
	auto sent = nothing_sent();
	sent[static_cast<std::size_t>(TecuMessage::WheelBasedSpeed)] = true;

	const auto plan = plan_tecu_messages(true, others, sent);
	for (std::size_t i = 0; i < TECU_MESSAGE_COUNT; i++)
	{
		check(plan.send[i], "primary sends every message, whatever others do");
	}
}

static void test_secondary_alone_sends_everything()
{
	std::printf("test_secondary_alone_sends_everything\n");
	const auto plan = plan_tecu_messages(false, Facilities{}, nothing_sent());
	for (std::size_t i = 0; i < TECU_MESSAGE_COUNT; i++)
	{
		check(plan.send[i], "secondary with nothing provided by others sends every message");
	}
}

// The ISO 11783-9 example: the tractor has wheel sensors but no ground-based speed; the display adds it from GNSS
static void test_secondary_adds_ground_speed_only()
{
	std::printf("test_secondary_adds_ground_speed_only\n");
	Facilities tractor;
	tractor.wheelBasedSpeed = true;
	tractor.wheelBasedDistance = true;
	tractor.wheelBasedDirection = true;
	tractor.selectedSpeed = true;
	tractor.timeDate = true;
	auto sent = nothing_sent();
	sent[static_cast<std::size_t>(TecuMessage::VehicleDirectionSpeed)] = true;

	const auto plan = plan_tecu_messages(false, tractor, sent);
	check(plan.sends(TecuMessage::GroundBasedSpeed), "sends ground-based speed (tractor lacks it)");
	check(!plan.sends(TecuMessage::WheelBasedSpeed), "leaves wheel-based speed to the tractor");
	check(!plan.sends(TecuMessage::MachineSelectedSpeed), "leaves machine selected speed to the tractor");
	check(!plan.sends(TecuMessage::TimeDate), "leaves time/date to the tractor");
	check(!plan.sends(TecuMessage::VehicleDirectionSpeed), "does not double a message the tractor sends without a facility bit");
	check(plan.sends(TecuMessage::CogSogRapidUpdate), "sends COG/SOG, which the tractor neither advertises nor sends");

	const Facilities ours = facilities_for_plan(plan);
	check(ours.tecuClass == 1, "advertises class 1");
	check(ours.groundBasedSpeed && ours.groundBasedDistance && ours.groundBasedDirection, "advertises ground-based speed, distance, direction");
	check(!ours.wheelBasedSpeed && !ours.wheelBasedDistance && !ours.wheelBasedDirection, "does not advertise wheel-based facilities");
	check(!ours.selectedSpeed, "does not advertise selected speed");
	check(!ours.timeDate, "time/date bit is left to the time/date code");

	// ISO 11783-9 4.4.2.1 d: no facility bit set by both
	const auto tractorPayload = encode_facilities(tractor);
	const auto ourPayload = encode_facilities(ours);
	for (std::size_t i = 1; i < tractorPayload.size(); i++)
	{
		check(0 == (tractorPayload[i] & ourPayload[i]), "no facility bit advertised by both TECUs");
	}
	check(0 == (tractorPayload[0] & ourPayload[0] & 0x3F), "no byte 1 facility bit advertised by both TECUs");
}

static void test_seen_on_bus_counts_without_facility_bit()
{
	std::printf("test_seen_on_bus_counts_without_facility_bit\n");
	auto sent = nothing_sent();
	sent[static_cast<std::size_t>(TecuMessage::GroundBasedSpeed)] = true; // sent but not advertised
	const auto plan = plan_tecu_messages(false, Facilities{}, sent);
	check(!plan.sends(TecuMessage::GroundBasedSpeed), "a message seen from another TECU is not sent, advertised or not");
	check(plan.sends(TecuMessage::WheelBasedSpeed), "others are still sent");
}

static void test_primary_advertises_selected_speed()
{
	std::printf("test_primary_advertises_selected_speed\n");
	const Facilities ours = facilities_for_plan(plan_tecu_messages(true, Facilities{}, nothing_sent()));
	const auto payload = encode_facilities(ours);
	check(payload[0] == 0x06, "byte 1: class 1, ground- and wheel-based speed");
	check(payload[2] == 0x78, "byte 3: ground- and wheel-based distance and direction");
	check(payload[4] == 0x04, "byte 5: selected speed (ISO 11783-7 B.24.3 byte 5 bit 3)");
}

static void test_merge_facilities()
{
	std::printf("test_merge_facilities\n");
	Facilities a;
	a.tecuClass = 1;
	a.wheelBasedSpeed = true;
	Facilities b;
	b.tecuClass = 2;
	b.timeDate = true;
	b.frontHitchPosition = true;
	const Facilities merged = merge_facilities(a, b);
	check(merged.wheelBasedSpeed && merged.timeDate && merged.frontHitchPosition, "merge keeps every facility of both");
	check(!merged.groundBasedSpeed, "merge does not invent facilities");
	check(merged.tecuClass == 2, "merge keeps the higher class");
}

static void test_pgn_from_can_identifier()
{
	std::printf("test_pgn_from_can_identifier\n");
	check(pgn_from_can_identifier(0x0CFE49F0) == 65097, "ground-based speed from 0xF0");
	check(pgn_from_can_identifier(0x0CFE4826) == 65096, "wheel-based speed from 0x26");
	check(pgn_from_can_identifier(0x0CF022F0) == 61474, "machine selected speed (PDU2, PS is part of the PGN)");
	check(pgn_from_can_identifier(0x18FEE6F0) == 65254, "time/date");
	check(pgn_from_can_identifier(0x09F80281) == 129026, "NMEA 2000 COG/SOG (data page 1)");
	check(pgn_from_can_identifier(0x18EA80F0) == 0xEA00, "PDU1 request: destination address is not part of the PGN");
	check(tecu_message_pgn(TecuMessage::CogSogRapidUpdate) == 129026, "COG/SOG PGN");
}

// A lower TECU that answered a Time/Date request keeps providing it until it leaves, however long the gap
static void test_time_date_providers()
{
	std::printf("test_time_date_providers\n");
	TimeDateProviders providers;
	check(!providers.any(), "nobody provides time/date at first");
	providers.mark_sent(0x80);
	check(providers.any(), "a TECU that sent time/date provides it");
	providers.forget(0x81);
	check(providers.any(), "another address leaving changes nothing");
	providers.forget(0x80);
	check(!providers.any(), "the provider leaving the bus ends it");
}

int main()
{
	test_primary_sends_everything();
	test_secondary_alone_sends_everything();
	test_secondary_adds_ground_speed_only();
	test_seen_on_bus_counts_without_facility_bit();
	test_primary_advertises_selected_speed();
	test_merge_facilities();
	test_pgn_from_can_identifier();
	test_time_date_providers();

	if (failures > 0)
	{
		std::fprintf(stderr, "\n%d check(s) FAILED\n", failures);
		return 1;
	}
	std::printf("\nAll checks passed\n");
	return 0;
}
