/**
 * @file guidance_track_context.hpp
 * @brief Abstraction layer between AOG input and ISOBUS TRACK (Generation 1) protocol.
 *
 * This header defines the GuidanceTrackContext struct and the GuidanceTrackProvider
 * that consumes AOG PGN 0xF4 guidance-track data.
 *
 * The ISOBUS TRACK sender consumes a GuidanceTrackContext without caring how it
 * was produced. The GuidanceTrackContext maps directly to ISOBUS DDIs 508–511.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <span>

/**
 * @brief Immutable snapshot of guidance-track state consumed by the ISOBUS TRACK sender.
 *
 * Corresponds to ISOBUS DDIs:
 *   - guidanceReferenceLineId -> DDI 508 (Unique Guidance Reference Line ID)
 *   - actualTrackNumber       -> DDI 509 (Actual Guidance Track Number)
 *   - trackNumberRight        -> DDI 510 (Guidance Track Number to the Right)
 *   - trackNumberLeft         -> DDI 511 (Guidance Track Number to the Left)
 *   - swathWidthMm            -> DDI 512 (Guidance Line Swath Width)
 */
struct GuidanceTrackContext
{
	std::uint32_t guidanceReferenceLineId = 1; ///< DDI 508
	std::int32_t actualTrackNumber = 0; ///< DDI 509 — signed; track 0 is valid
	std::int32_t trackNumberRight = -1; ///< DDI 510
	std::int32_t trackNumberLeft = 1; ///< DDI 511
	std::uint32_t swathWidthMm = 0; ///< DDI 512 � distance between adjacent tracks; 0 = not reported by AOG
	bool valid = false; ///< Context has been initialized with at least one update
};

/**
 * @brief Real guidance-track provider that consumes AOG PGN 0xF4 (244) data.
 *
 * PGN 0xF4 payload layout (10 data bytes, optionally 12):
 *   Byte 0: Sequence counter (0–255, wrapping)
 *   Byte 1: Flags (bit 0 = valid, bit 1 = heading same way, bit 2 = curve mode)
 *   Bytes 2-3: Guidance Reference ID (uint16 LE)
 *   Bytes 4-5: Current Track Number (int16 LE, signed)
 *   Bytes 6-7: Track Number Left (int16 LE, signed)
 *   Bytes 8-9: Track Number Right (int16 LE, signed)
 *   Bytes 10-11: Swath Width in mm (uint16 LE), optional � only present from newer AOG builds
 *
 * Track numbers (current/left/right) are each shifted by +1 from the raw wire value
 * (see AOG_TRACK_NUMBER_OFFSET in parse()) to match the tram-pattern phase implements
 * expect — confirmed by field testing, not part of AOG's own documented wire format.
 *
 * Sequence tracking:
 *   - Rejects any packet whose sequence number is not strictly ahead of the last
 *     accepted one (signed delta over the 0-255 wrap), catching both frozen/duplicate
 *     data and reordered/stale UDP packets arriving out of order.
 *   - Call reset() after a disconnect to treat the next packet as a fresh start.
 *
 * Returns valid=true when data is valid, refId != 0, and sequence is fresh.
 */
class GuidanceTrackProvider
{
public:
	/// Minimum payload size (10 data bytes)
	static constexpr std::size_t MIN_PAYLOAD_SIZE = 10;

	/// Payload size that includes the swath width
	static constexpr std::size_t PAYLOAD_SIZE_WITH_SWATH_WIDTH = 12;

	/**
	 * @brief Parse AOG PGN 0xF4 payload and produce a GuidanceTrackContext.
	 *
	 * @param data Payload bytes (after UDP header stripping)
	 * @return GuidanceTrackContext with valid=true if parse succeeded and flags indicate valid data
	 */
	GuidanceTrackContext parse(std::span<const std::uint8_t> data);

	/// @brief Reset sequence tracking (e.g., after AOG disconnect timeout).
	/// The next parse() call is treated as a fresh start — no delta comparison.
	void reset();

private:
	std::optional<std::uint8_t> lastSequence_; ///< Unset until the first packet is accepted
};
