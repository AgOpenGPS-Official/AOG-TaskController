/**
 * @brief Unit tests for GuidanceTrackProvider, which parses AOG's PGN 0xF4.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "guidance_track_context.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
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

static std::array<std::uint8_t, 10> make_packet(std::uint8_t sequence,
                                                std::uint8_t flags,
                                                std::uint16_t referenceId,
                                                std::int16_t current,
                                                std::int16_t left,
                                                std::int16_t right)
{
	auto lo = [](std::int16_t v) { return static_cast<std::uint8_t>(static_cast<std::uint16_t>(v) & 0xFF); };
	auto hi = [](std::int16_t v) { return static_cast<std::uint8_t>((static_cast<std::uint16_t>(v) >> 8) & 0xFF); };
	return { sequence,
		       flags,
		       static_cast<std::uint8_t>(referenceId & 0xFF),
		       static_cast<std::uint8_t>(referenceId >> 8),
		       lo(current),
		       hi(current),
		       lo(left),
		       hi(left),
		       lo(right),
		       hi(right) };
}

static GuidanceTrackContext parse(GuidanceTrackProvider &provider, const std::array<std::uint8_t, 10> &packet)
{
	return provider.parse(std::span<const std::uint8_t>(packet.data(), packet.size()));
}

static GuidanceTrackContext parse_with_swath_width(GuidanceTrackProvider &provider, std::uint16_t swathWidthMm)
{
	auto base = make_packet(0, 0x01, 1, 0, -1, 1);
	std::array<std::uint8_t, 12> packet{};
	std::copy(base.begin(), base.end(), packet.begin());
	packet[10] = static_cast<std::uint8_t>(swathWidthMm & 0xFF);
	packet[11] = static_cast<std::uint8_t>(swathWidthMm >> 8);
	return provider.parse(std::span<const std::uint8_t>(packet.data(), packet.size()));
}

static void test_valid_packet_fields_and_offset()
{
	std::printf("test_valid_packet_fields_and_offset\n");

	GuidanceTrackProvider provider;
	auto ctx = parse(provider, make_packet(0, 0x01, 0x1234, 5, 4, 6));

	check(ctx.valid, "valid packet is accepted");
	check(ctx.guidanceReferenceLineId == 0x1234, "reference line ID is passed through");
	check(ctx.actualTrackNumber == 6, "actual track number gets the +1 offset");
	check(ctx.trackNumberLeft == 5, "left track number gets the +1 offset");
	check(ctx.trackNumberRight == 7, "right track number gets the +1 offset");
}

static void test_swath_width()
{
	std::printf("test_swath_width\n");

	GuidanceTrackProvider provider;
	auto ctx = parse_with_swath_width(provider, 18000);
	check(ctx.valid, "12-byte packet is accepted");
	check(ctx.swathWidthMm == 18000, "swath width is decoded as little-endian mm");

	GuidanceTrackProvider legacyProvider;
	auto legacy = parse(legacyProvider, make_packet(0, 0x01, 1, 0, -1, 1));
	check(legacy.valid, "10-byte packet is still accepted");
	check(legacy.swathWidthMm == 0, "10-byte packet reports no swath width");
}

static void test_negative_track_numbers()
{
	std::printf("test_negative_track_numbers\n");

	GuidanceTrackProvider provider;
	auto ctx = parse(provider, make_packet(0, 0x01, 1, -3, -4, -2));

	check(ctx.valid, "negative track numbers are valid");
	check(ctx.actualTrackNumber == -2, "-3 + offset = -2");
	check(ctx.trackNumberLeft == -3, "-4 + offset = -3");
	check(ctx.trackNumberRight == -1, "-2 + offset = -1");
}

static void test_short_payload_rejected()
{
	std::printf("test_short_payload_rejected\n");

	GuidanceTrackProvider provider;
	const std::array<std::uint8_t, 9> shortPacket = { 0, 0x01, 1, 0, 0, 0, 0, 0, 0 };
	auto ctx = provider.parse(std::span<const std::uint8_t>(shortPacket.data(), shortPacket.size()));

	check(!ctx.valid, "a 9-byte payload is rejected");
}

static void test_guidance_off()
{
	std::printf("test_guidance_off\n");

	GuidanceTrackProvider provider;
	check(!parse(provider, make_packet(0, 0x00, 7, 1, 0, 2)).valid, "valid flag clear -> invalid");
	check(!parse(provider, make_packet(1, 0x01, 0, 1, 0, 2)).valid, "reference line ID 0 -> invalid");
	check(parse(provider, make_packet(2, 0x01, 7, 1, 0, 2)).valid, "a guidance-off packet still advances the sequence");
}

static void test_sequence_freshness()
{
	std::printf("test_sequence_freshness\n");

	GuidanceTrackProvider provider;
	check(parse(provider, make_packet(10, 0x01, 1, 0, 0, 0)).valid, "first packet is accepted");
	check(!parse(provider, make_packet(10, 0x01, 1, 0, 0, 0)).valid, "duplicate sequence is rejected");
	check(!parse(provider, make_packet(9, 0x01, 1, 0, 0, 0)).valid, "older sequence is rejected");
	check(parse(provider, make_packet(11, 0x01, 1, 0, 0, 0)).valid, "next sequence is accepted");
}

static void test_sequence_wrap()
{
	std::printf("test_sequence_wrap\n");

	GuidanceTrackProvider provider;
	check(parse(provider, make_packet(255, 0x01, 1, 0, 0, 0)).valid, "255 is accepted");
	check(parse(provider, make_packet(0, 0x01, 1, 0, 0, 0)).valid, "0 after 255 is forward progress");
	check(!parse(provider, make_packet(255, 0x01, 1, 0, 0, 0)).valid, "255 after 0 is a step back");
}

static void test_reset()
{
	std::printf("test_reset\n");

	GuidanceTrackProvider provider;
	check(parse(provider, make_packet(50, 0x01, 1, 0, 0, 0)).valid, "packet accepted");
	provider.reset();
	check(parse(provider, make_packet(50, 0x01, 1, 0, 0, 0)).valid, "same sequence is accepted again after reset");
}

int main()
{
	test_valid_packet_fields_and_offset();
	test_swath_width();
	test_negative_track_numbers();
	test_short_payload_rejected();
	test_guidance_off();
	test_sequence_freshness();
	test_sequence_wrap();
	test_reset();

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("All guidance track tests passed\n");
	return 0;
}
