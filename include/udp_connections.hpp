/**
 * @author Daan Steenbergen
 * @brief UDP connections to communicate with AgOpenGPS
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include <boost/asio.hpp>
#include <span>
#include "settings.hpp"

using boost::asio::ip::udp;

/// @brief A callback interface for handling incoming packets
/// @param src The source of the packet
/// @param pgn The PGN of the packet
/// @param data The data of the packet
using PacketCallback = std::function<void(std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data)>;

/// @brief UDP connections to communicate with AgOpenGPS
class UdpConnections
{
public:
	/**
      * @brief Construct a new UDPConnections object
      * @param settings The settings to use
      * @param ioContext The IO context to use
      */
	UdpConnections(std::shared_ptr<Settings> settings, boost::asio::io_context &ioContext);

	/**
      * @brief Set packet handler
      * @param packetCallback The callback to use for incoming packets
      */
	void set_packet_handler(PacketCallback packetCallback);

	/**
     * @brief Open the UDP connections
     * @param endpoint The endpoint to open the connection on
     * @return True if the connections were opened successfully, false otherwise
     */
	bool open();

	/**
     * @brief Close the UDP connections
     */
	void close();

	/**
     * @brief Handle incoming packets
     */
	void handle_incoming_packets();

	/**
     * @brief Handle address detection
     */
	void handle_address_detection();

	/**
     * @brief Send packet to AOG
     * @param src The source of the packet
     * @param pgn The PGN of the packet
     * @param data The data of the packet
     * @return True if the packet was sent successfully, false otherwise
     */
	bool send(std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data);

	/**
	 * @brief Get the IPv4 address used by the AOG UDP connection
	 * @return The bound address, or "not available" if the socket is not open
	 */
	std::string get_bound_ip_address() const;

private:
	static const std::size_t MAX_PACKET_SIZE = 512; // Mostly arbitrary, but should be large enough to hold any packet
	static const std::uint16_t PACKET_START = 0x8081; // Start of packet

	/**
     * @brief Get the local endpoint
     * @return The local endpoint
     */
	udp::endpoint get_local_endpoint() const;

	/**
      * @brief Calculate CRC for data
      * @param data The data to calculate the CRC for
      * @return The calculated CRC
      */
	std::uint8_t calculate_crc(std::span<std::uint8_t> data);

	PacketCallback packetCallback = nullptr;
	std::shared_ptr<Settings> settings;
	udp::socket udpConnection;
	udp::socket udpConnectionAddressDetection;
};
