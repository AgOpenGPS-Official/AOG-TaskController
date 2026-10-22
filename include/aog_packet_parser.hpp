/**
 * @brief Splits one AgIO/AOG UDP datagram into its packets
 *
 * A packet is: 0x80 0x81, source, PGN, length, <length> data bytes, CRC.
 * AgIO never splits a packet across datagrams, so each datagram is parsed on its own
 * and nothing is carried over to the next one.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace aog_packet
{
	inline constexpr std::uint8_t START_HIGH = 0x80;
	inline constexpr std::uint8_t START_LOW = 0x81;
	inline constexpr std::size_t HEADER_SIZE = 5; ///< Start (2), source, PGN, length
	inline constexpr std::size_t CRC_SIZE = 1;

	/// @brief Why parsing a datagram stopped before its end
	enum class ParseStop
	{
		Complete, ///< Every byte belonged to a packet
		UnknownStart, ///< Bytes at the stop offset do not start with 0x80 0x81
		Truncated ///< A packet header claims more bytes than the datagram holds
	};

	struct ParseResult
	{
		ParseStop stop = ParseStop::Complete;
		std::size_t offset = 0; ///< Where parsing stopped; equals the datagram size when complete
		std::size_t packets = 0; ///< Number of packets handed to the callback
	};

	/**
	 * @brief Hands every complete packet in a datagram to a callback, in order
	 * @param datagram The bytes of one received datagram
	 * @param onPacket Called as onPacket(src, pgn, data) for each packet; data excludes the CRC
	 * @return Where and why parsing stopped
	 */
	template<typename Callback>
	ParseResult parse_datagram(std::span<std::uint8_t> datagram, Callback &&onPacket)
	{
		ParseResult result;
		while (result.offset < datagram.size())
		{
			const std::size_t remaining = datagram.size() - result.offset;
			if (remaining < HEADER_SIZE + CRC_SIZE)
			{
				result.stop = ParseStop::Truncated;
				return result;
			}

			const std::uint8_t *packet = datagram.data() + result.offset;
			if ((START_HIGH != packet[0]) || (START_LOW != packet[1]))
			{
				result.stop = ParseStop::UnknownStart;
				return result;
			}

			const std::uint8_t src = packet[2];
			const std::uint8_t pgn = packet[3];
			const std::size_t length = packet[4];
			if (remaining < HEADER_SIZE + length + CRC_SIZE)
			{
				result.stop = ParseStop::Truncated;
				return result;
			}

			onPacket(src, pgn, datagram.subspan(result.offset + HEADER_SIZE, length));
			result.offset += HEADER_SIZE + length + CRC_SIZE;
			result.packets++;
		}
		return result;
	}
} // namespace aog_packet
