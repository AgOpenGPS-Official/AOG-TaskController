/**
 * @file guidance_track_context.cpp
 * @brief Implementation of GuidanceTrackProvider, see guidance_track_context.hpp.
 */

#include "guidance_track_context.hpp"
#include "logging_utils.hpp"

#include <iostream>

namespace
{
	/// @brief Decode a little-endian uint16 from a 2-byte span starting at offset.
	std::uint16_t decode_le_u16(std::span<const std::uint8_t> data, std::size_t offset)
	{
		return static_cast<std::uint16_t>(data[offset]) |
		  (static_cast<std::uint16_t>(data[offset + 1]) << 8);
	}

	/// @brief Decode a little-endian, signed int16 from a 2-byte span starting at offset.
	std::int16_t decode_le_i16(std::span<const std::uint8_t> data, std::size_t offset)
	{
		return static_cast<std::int16_t>(decode_le_u16(data, offset));
	}
} // namespace

GuidanceTrackContext GuidanceTrackProvider::parse(std::span<const std::uint8_t> data)
{
	GuidanceTrackContext ctx;

	if (data.size() < MIN_PAYLOAD_SIZE)
	{
		std::cout << "[" << get_timestamp() << "] [TRACK][real] PGN 0xF4 too short (len="
		          << data.size() << ")" << std::endl;
		return ctx;
	}

	// Parse fields
	std::uint8_t sequence = data[0];
	std::uint8_t flags = data[1];
	bool isValid = (flags & 0x01) != 0;
	bool headingSameWay = (flags & 0x02) != 0;
	bool curveMode = (flags & 0x04) != 0;

	std::uint16_t refId = decode_le_u16(data, 2);

	// AOG's raw track-number wire convention is one pass off from what a tramline
	// implement's own on-board phase computation (e.g. "which of N passes is this")
	// expects, for both left and right passes. Confirmed via field testing: with the
	// raw value relayed as-is, the sprayer's ON/OFF tram state was consistently one
	// pass out of phase with AOG's own intended tram state; shifting every value by
	// +1 (independent of sign) brought them into agreement across every pass tested.
	static constexpr std::int16_t AOG_TRACK_NUMBER_OFFSET = 1;
	std::int16_t currentTrack = static_cast<std::int16_t>(decode_le_i16(data, 4) + AOG_TRACK_NUMBER_OFFSET);
	std::int16_t trackLeft = static_cast<std::int16_t>(decode_le_i16(data, 6) + AOG_TRACK_NUMBER_OFFSET);
	std::int16_t trackRight = static_cast<std::int16_t>(decode_le_i16(data, 8) + AOG_TRACK_NUMBER_OFFSET);

	// Sequence freshness check: signed delta over the 0-255 wrap must be strictly
	// positive (forward progress). Rejects both frozen/duplicate packets (delta == 0)
	// and reordered/stale packets that arrived out of order (delta < 0).
	const char *outcome;

	if (lastSequence_.has_value() && static_cast<std::int8_t>(sequence - *lastSequence_) <= 0)
	{
		outcome = "REJECTED (stale/duplicate/out-of-order sequence)";
	}
	else
	{
		lastSequence_ = sequence;

		if (!isValid || refId == 0)
		{
			outcome = "guidance OFF (no active track)";
		}
		else
		{
			ctx.guidanceReferenceLineId = refId;
			ctx.actualTrackNumber = currentTrack;
			ctx.trackNumberLeft = trackLeft;
			ctx.trackNumberRight = trackRight;
			ctx.valid = true;
			outcome = "ACCEPTED";
		}
	}

	std::cout << "[" << get_timestamp() << "] [TRACK][real] seq=" << static_cast<int>(sequence)
	          << " flags=0x" << std::hex << static_cast<int>(flags) << std::dec
	          << " (valid=" << isValid << " sameHeading=" << headingSameWay << " curve=" << curveMode << ")"
	          << " ref=" << refId
	          << " left=" << trackLeft << " actual=" << currentTrack << " right=" << trackRight
	          << " -> " << outcome << std::endl;

	return ctx;
}

void GuidanceTrackProvider::reset()
{
	lastSequence_.reset();
}
