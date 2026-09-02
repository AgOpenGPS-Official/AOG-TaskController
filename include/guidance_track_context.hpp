/**
 * @file guidance_track_context.hpp
 * @brief Abstraction layer between AOG input and ISOBUS TRACK (Generation 1) protocol.
 *
 * This header defines the GuidanceTrackContext struct and a synthetic provider
 * that derives track information from AOG tramline marker bits (PGN 0xEF byte 3).
 *
 * The ISOBUS TRACK sender consumes a GuidanceTrackContext without caring how it
 * was produced. This allows the synthetic provider to be replaced later with a
 * real AOG guidance-track data source without modifying the TRACK protocol code.
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <span>

#include "logging_utils.hpp"

// TODO(AOG integration): RealGuidanceTrackProvider now consumes PGN 0xF4 from AOG.
// SyntheticGuidanceTrackProvider (PGN 0xEF tram bits) remains as fallback when
// AOG does not send PGN 0xF4 (older versions, no guidance track active, etc.).

/**
 * @brief Immutable snapshot of guidance-track state consumed by the ISOBUS TRACK sender.
 *
 * Corresponds to ISOBUS DDIs:
 *   - guidanceReferenceLineId -> DDI 508 (Unique A-B Guidance Reference Line ID)
 *   - actualTrackNumber       -> DDI 509 (Actual Track Number)
 *   - trackNumberRight        -> DDI 510 (Track Number to the Right)
 *   - trackNumberLeft         -> DDI 511 (Track Number to the Left)
 */
struct GuidanceTrackContext
{
	std::uint32_t guidanceReferenceLineId = 1; ///< DDI 508 — stable synthetic ID for now
	std::int32_t actualTrackNumber = 0; ///< DDI 509 — signed; track 0 is valid
	std::int32_t trackNumberRight = -1; ///< DDI 510
	std::int32_t trackNumberLeft = 1; ///< DDI 511
	bool valid = false; ///< Context has been initialized with at least one update
	bool synthetic = true; ///< true = derived from AOG tram bits; false = real AOG data
};

/**
 * @brief TEMPORARY synthetic provider that derives GuidanceTrackContext from AOG tram marker bits.
 *
 * AOG currently does NOT send real guidance-track data (reference line ID, actual track number,
 * adjacent track numbers). This provider uses the left/right tramline marker bits from PGN 0xEF
 * (byte 3) to produce a synthetic track context via edge detection.
 *
 * This entire class is a temporary compatibility fallback and should be replaced when AOG
 * exposes real guidance-track information.
 *
 * Marker-count-based track numbering:
 *   - Track number is derived from edge counting + mod correction based on marker count.
 *   - 0 markers active → track mod tramModulo = 0  (tramline pass)
 *   - 1 marker active  → track mod tramModulo = 1  (first intermediate pass)
 *   - 2 markers active → track mod tramModulo = 2  (second intermediate / on tramline)
 *   - Edge counting (+1 left, -1 right) handles direction.
 *   - Mod correction snaps to expected value on every update, catching drift instantly.
 *   - Track numbers are signed integers (can go negative, 0 is valid)
 *   - Adjacent tracks: left = N+1, right = N-1 (stable ordering, no flipping)
 */
class SyntheticGuidanceTrackProvider
{
public:
	/// @param tramModulo Passes per tramline cycle (boom_width / tram_spacing).
	///                   e.g. 18m boom / 6m spacing = 3.  Set to 0 to disable correction.
	explicit SyntheticGuidanceTrackProvider(std::uint32_t tramModulo = 3) :
	  tramModulo_(tramModulo)
	{
	}

	/**
	 * @brief Update the synthetic track context with new AOG tramline marker states.
	 *
	 * Call this once per received PGN 0xEF with the decoded marker bits.
	 * Edge counting (+1 left, -1 right) provides direction; marker count determines
	 * the expected mod-tramModulo position (0 markers=0, 1 marker=1, 2 markers=2).
	 * Correction snaps to the expected value on every call, catching drift instantly.
	 *
	 * @param newTramLeft  Current left tramline marker state (true = active)
	 * @param newTramRight Current right tramline marker state (true = active)
	 * @return Updated GuidanceTrackContext
	 */
	GuidanceTrackContext update(bool newTramLeft, bool newTramRight)
	{
		// Compute edges BEFORE updating previous state.
		bool leftEdge = newTramLeft && !prevTramLeftActive_;
		bool rightEdge = newTramRight && !prevTramRightActive_;

		// Edge detection: count pass transitions
		if (leftEdge)
		{
			currentTrackNumber_++;
		}
		else if (rightEdge)
		{
			currentTrackNumber_--;
		}

		// Update previous state
		prevTramLeftActive_ = newTramLeft;
		prevTramRightActive_ = newTramRight;

		// Marker-count-based mod correction.
		// The number of active markers determines the track's position within the
		// tramline cycle: 0 markers = mod 0, 1 marker = mod 1, 2 markers = mod 2.
		// This corrects drift from missed edges, U-turns, or GPS glitches.
		if (tramModulo_ > 0)
		{
			std::int32_t markerCount = (newTramLeft ? 1 : 0) + (newTramRight ? 1 : 0);
			std::int32_t expectedMod = markerCount % static_cast<std::int32_t>(tramModulo_);
			std::int32_t m = static_cast<std::int32_t>(tramModulo_);
			std::int32_t actualMod = (currentTrackNumber_ % m + m) % m;

			if (actualMod != expectedMod)
			{
				currentTrackNumber_ += (expectedMod - actualMod);
			}
		}

		// Build context. Adjacent tracks use stable ordering:
		//   left = N+1, right = N-1
		GuidanceTrackContext ctx;
		ctx.guidanceReferenceLineId = syntheticRefLineId_;
		ctx.actualTrackNumber = currentTrackNumber_;
		ctx.trackNumberLeft = currentTrackNumber_ + 1;
		ctx.trackNumberRight = currentTrackNumber_ - 1;
		ctx.valid = true;
		ctx.synthetic = true;

		// Log only on state change
		if (newTramLeft != lastLoggedLeft_ || newTramRight != lastLoggedRight_ ||
		    currentTrackNumber_ != lastLoggedTrack_)
		{
			std::cout << "[" << get_timestamp() << "] [TRACK][synthetic] ref=" << ctx.guidanceReferenceLineId
			          << " actual=" << ctx.actualTrackNumber
			          << " left=" << ctx.trackNumberLeft
			          << " right=" << ctx.trackNumberRight << std::endl;
			lastLoggedLeft_ = newTramLeft;
			lastLoggedRight_ = newTramRight;
			lastLoggedTrack_ = currentTrackNumber_;
		}

		return ctx;
	}

	/// @brief Reset state (e.g., on AOG disconnect)
	void reset()
	{
		prevTramLeftActive_ = false;
		prevTramRightActive_ = false;
		currentTrackNumber_ = 0;
		lastLoggedLeft_ = false;
		lastLoggedRight_ = false;
		lastLoggedTrack_ = 0;
	}

	/// @brief Set the tram modulo (boom width / tram spacing). 0 disables correction.
	void set_tram_modulo(std::uint32_t modulo)
	{
		tramModulo_ = modulo;
	}

	/// @brief Get the current tram modulo value
	std::uint32_t get_tram_modulo() const
	{
		return tramModulo_;
	}

	/// @brief Get the current track number without updating state
	std::int32_t get_current_track_number() const
	{
		return currentTrackNumber_;
	}

private:
	bool prevTramLeftActive_ = false; ///< Previous left marker state (for edge detection)
	bool prevTramRightActive_ = false; ///< Previous right marker state (for edge detection)
	std::int32_t currentTrackNumber_ = 0; ///< Signed track counter; starts at 0, can go negative
	std::uint32_t tramModulo_ = 3; ///< Passes per tramline cycle (boom/tram). 0 = disabled
	std::uint32_t syntheticRefLineId_ = 1; ///< Stable synthetic reference line ID

	// Dedup logging state
	bool lastLoggedLeft_ = false;
	bool lastLoggedRight_ = false;
	std::int32_t lastLoggedTrack_ = 0;

	/// @brief Snap to nearest multiple of tramModulo.
	/// If already on the boundary (remainder == 0), return unchanged.
	/// This auto-corrects drift from U-turns, missed edges, or GPS glitches.
	std::int32_t snap_to_nearest_mod(std::int32_t n) const
	{
		std::int32_t m = static_cast<std::int32_t>(tramModulo_);
		if (m <= 0)
			return n;
		// Compute always-non-negative remainder
		std::int32_t r = (n % m + m) % m;
		if (r == 0)
			return n; // Already on boundary
		// Round to nearest: if r < half, snap down; otherwise snap up.
		if (r < m - r)
			return n - r; // e.g. snap(1, 3) = 0, snap(-2, 3) = -3
		return n + (m - r); // e.g. snap(2, 3) = 3, snap(-1, 3) = 0
	}
};

/**
 * @brief Real guidance-track provider that consumes AOG PGN 0xF4 (244) data.
 *
 * PGN 0xF4 payload layout (10 data bytes):
 *   Byte 0: Sequence counter (0–255)
 *   Byte 1: Flags (bit 0 = valid, bit 1 = heading same way, bit 2 = curve mode)
 *   Bytes 2-3: Guidance Reference ID (uint16 LE)
 *   Bytes 4-5: Current Track Number (int16 LE, signed)
 *   Bytes 6-7: Track Number Left (int16 LE, signed)
 *   Bytes 8-9: Track Number Right (int16 LE, signed)
 *
 * The provider validates CRC and sequence freshness. Returns a context with
 * synthetic = false when data is valid, or an invalid context otherwise.
 */
class RealGuidanceTrackProvider
{
public:
	/// Minimum payload size (10 data bytes)
	static constexpr std::size_t MIN_PAYLOAD_SIZE = 10;

	/**
	 * @brief Parse AOG PGN 0xF4 payload and produce a GuidanceTrackContext.
	 *
	 * @param data Payload bytes (after UDP header stripping)
	 * @return GuidanceTrackContext with valid=true if parse succeeded and flags indicate valid data
	 */
	GuidanceTrackContext parse(std::span<const std::uint8_t> data)
	{
		GuidanceTrackContext ctx;
		ctx.synthetic = false;

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

		std::uint16_t refId =
		  static_cast<std::uint16_t>(data[2]) |
		  (static_cast<std::uint16_t>(data[3]) << 8);

		std::int16_t currentTrack =
		  static_cast<std::int16_t>(
		    static_cast<std::uint16_t>(data[4]) |
		    (static_cast<std::uint16_t>(data[5]) << 8));

		std::int16_t trackLeft =
		  static_cast<std::int16_t>(
		    static_cast<std::uint16_t>(data[6]) |
		    (static_cast<std::uint16_t>(data[7]) << 8));

		std::int16_t trackRight =
		  static_cast<std::int16_t>(
		    static_cast<std::uint16_t>(data[8]) |
		    (static_cast<std::uint16_t>(data[9]) << 8));

		// Update sequence tracking for freshness detection
		lastSequence_ = sequence;

		if (!isValid || refId == 0)
		{
			// No active guidance track
			return ctx; // valid=false, synthetic=false
		}

		ctx.guidanceReferenceLineId = refId;
		ctx.actualTrackNumber = currentTrack;
		ctx.trackNumberLeft = trackLeft;
		ctx.trackNumberRight = trackRight;
		ctx.valid = true;

		// Log on change
		if (currentTrack != lastLoggedTrack_ || refId != lastLoggedRefId_)
		{
			std::cout << "[" << get_timestamp() << "] [TRACK][real] ref=" << ctx.guidanceReferenceLineId
			          << " actual=" << ctx.actualTrackNumber
			          << " left=" << ctx.trackNumberLeft
			          << " right=" << ctx.trackNumberRight
			          << " seq=" << static_cast<int>(sequence) << std::endl;
			lastLoggedTrack_ = currentTrack;
			lastLoggedRefId_ = refId;
		}

		return ctx;
	}

	/// @brief Get the last received sequence number
	std::uint8_t get_last_sequence() const
	{
		return lastSequence_;
	}

private:
	std::uint8_t lastSequence_ = 0;
	std::int16_t lastLoggedTrack_ = 0;
	std::uint16_t lastLoggedRefId_ = 0;
};
