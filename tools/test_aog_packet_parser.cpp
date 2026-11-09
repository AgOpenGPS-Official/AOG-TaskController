/**
 * @brief Unit tests for aog_packet::parse_datagram, which splits an AgIO/AOG UDP datagram into packets.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "aog_packet_parser.hpp"

#include <cstdio>
#include <vector>

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

struct Packet
{
	std::uint8_t src;
	std::uint8_t pgn;
	std::vector<std::uint8_t> data;
};

static std::vector<std::uint8_t> make_packet(std::uint8_t pgn, std::vector<std::uint8_t> data)
{
	std::vector<std::uint8_t> bytes = { 0x80, 0x81, 0x7F, pgn, static_cast<std::uint8_t>(data.size()) };
	bytes.insert(bytes.end(), data.begin(), data.end());
	bytes.push_back(0x00); // CRC, not checked
	return bytes;
}

static aog_packet::ParseResult parse(std::vector<std::uint8_t> &datagram, std::vector<Packet> &packets)
{
	return aog_packet::parse_datagram(datagram, [&packets](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		packets.push_back({ src, pgn, std::vector<std::uint8_t>(data.begin(), data.end()) });
	});
}

static void test_short_packet_is_parsed_on_its_own()
{
	std::printf("test_short_packet_is_parsed_on_its_own\n");

	// PGN 241 (section control request) is 7 bytes; it must not wait for the next datagram.
	auto datagram = make_packet(0xF1, { 0x01 });
	std::vector<Packet> packets;
	auto result = parse(datagram, packets);

	check(aog_packet::ParseStop::Complete == result.stop, "a 7-byte datagram parses completely");
	check(1 == packets.size(), "one packet is delivered");
	check(!packets.empty() && 0xF1 == packets[0].pgn && 1 == packets[0].data.size() && 0x01 == packets[0].data[0], "PGN and data are correct");
}

static void test_short_datagram_does_not_eat_the_next_one()
{
	std::printf("test_short_datagram_does_not_eat_the_next_one\n");

	// The field failure: a 7-byte PGN 241 was kept in the buffer, the next datagram (PGN 229) was
	// appended to it, and the parser then read the second packet from the wrong offset and dropped it.
	auto first = make_packet(0xF1, { 0x01 });
	auto second = make_packet(0xE5, { 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
	std::vector<Packet> packets;
	parse(first, packets);
	auto result = parse(second, packets);

	check(aog_packet::ParseStop::Complete == result.stop, "the following datagram parses completely");
	check(2 == packets.size(), "both packets are delivered");
	check(2 == packets.size() && 0xE5 == packets[1].pgn && 8 == packets[1].data.size() && 0x03 == packets[1].data[0], "the PGN 229 section bits arrive intact");
}

static void test_several_packets_in_one_datagram()
{
	std::printf("test_several_packets_in_one_datagram\n");

	auto datagram = make_packet(0xF1, { 0x01 });
	auto second = make_packet(0xE5, { 0xFF, 0x0F, 0, 0, 0, 0, 0, 0 });
	auto third = make_packet(0xF2, { 0x8D, 0x01, 1, 2, 3, 4 });
	datagram.insert(datagram.end(), second.begin(), second.end());
	datagram.insert(datagram.end(), third.begin(), third.end());
	std::vector<Packet> packets;
	auto result = parse(datagram, packets);

	check(aog_packet::ParseStop::Complete == result.stop, "parses completely");
	check(3 == packets.size(), "all three packets are delivered");
	check(3 == packets.size() && 0xF1 == packets[0].pgn && 0xE5 == packets[1].pgn && 0xF2 == packets[2].pgn, "in order");
	check(3 == packets.size() && 0x0F == packets[1].data[1] && 4 == packets[2].data[5], "with their own data");
}

static void test_unknown_start_stops_without_delivering_garbage()
{
	std::printf("test_unknown_start_stops_without_delivering_garbage\n");

	auto datagram = make_packet(0xE5, { 1, 0, 0, 0, 0, 0, 0, 0 });
	datagram.insert(datagram.end(), { 0x24, 0x47, 0x50, 0x47, 0x47, 0x41, 0x2C }); // "$GPGGA," after a valid packet
	std::vector<Packet> packets;
	auto result = parse(datagram, packets);

	check(aog_packet::ParseStop::UnknownStart == result.stop, "stops at the unknown start");
	check(14 == result.offset, "reports where it stopped");
	check(1 == packets.size(), "the valid packet before it is still delivered");
}

static void test_truncated_packet_is_not_delivered()
{
	std::printf("test_truncated_packet_is_not_delivered\n");

	auto datagram = make_packet(0xE5, { 1, 0, 0, 0, 0, 0, 0, 0 });
	datagram.resize(datagram.size() - 3);
	std::vector<Packet> packets;
	auto result = parse(datagram, packets);

	check(aog_packet::ParseStop::Truncated == result.stop, "reports a truncated packet");
	check(packets.empty(), "delivers nothing past the end of the datagram");

	std::vector<std::uint8_t> headerOnly = { 0x80, 0x81, 0x7F };
	packets.clear();
	result = parse(headerOnly, packets);
	check(aog_packet::ParseStop::Truncated == result.stop, "a partial header is truncated, not read past");
	check(packets.empty(), "and delivers nothing");
}

static void test_empty_packet()
{
	std::printf("test_empty_packet\n");

	auto datagram = make_packet(0xC8, {});
	std::vector<Packet> packets;
	auto result = parse(datagram, packets);

	check(aog_packet::ParseStop::Complete == result.stop, "a zero-length packet parses");
	check(1 == packets.size() && packets[0].data.empty(), "with empty data");
}

int main()
{
	test_short_packet_is_parsed_on_its_own();
	test_short_datagram_does_not_eat_the_next_one();
	test_several_packets_in_one_datagram();
	test_unknown_start_stops_without_delivering_garbage();
	test_truncated_packet_is_not_delivered();
	test_empty_packet();

	if (0 != failures)
	{
		std::fprintf(stderr, "%d assertion(s) failed\n", failures);
		return 1;
	}
	std::printf("All tests passed\n");
	return 0;
}
