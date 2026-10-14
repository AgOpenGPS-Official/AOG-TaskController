/**
 * @author Daan Steenbergen
 * @brief UDP connections to communicate with AgOpenGPS
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */

#include "udp_connections.hpp"
#include <cassert>
#include <iostream>
#include "aog_packet_parser.hpp"
#include "logging_utils.hpp"

#if !defined(_WIN32)
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#endif

UdpConnections::UdpConnections(std::shared_ptr<Settings> settings, boost::asio::io_context &ioContext) :
  settings(settings),
  udpConnection(ioContext),
  udpConnectionAddressDetection(ioContext)
{
	assert(settings && "Settings must not be null");
}
void UdpConnections::set_packet_handler(PacketCallback packetCallback)
{
	this->packetCallback = packetCallback;
}

bool UdpConnections::open()
{
	// Set up the UDP server
	udpConnection.open(udp::v4());
	udpConnection.set_option(boost::asio::socket_base::broadcast(true));
	udpConnection.set_option(boost::asio::socket_base::reuse_address(true));
	udpConnection.non_blocking(true);
	udpConnection.bind(get_local_endpoint());

	// Set up another UDP server for Address Detection
	udpConnectionAddressDetection.open(udp::v4());
	udpConnectionAddressDetection.set_option(boost::asio::socket_base::broadcast(true));
	udpConnectionAddressDetection.set_option(boost::asio::socket_base::reuse_address(true));
	udpConnectionAddressDetection.non_blocking(true);
	udp::endpoint local_any_endpoint(boost::asio::ip::address_v4::any(), 8888); // Bind to 0.0.0.0 to receive packets on all interfaces
	udpConnectionAddressDetection.bind(local_any_endpoint);

	return true;
}

void UdpConnections::close()
{
	udpConnection.close();
	udpConnectionAddressDetection.close();
}

std::string UdpConnections::get_bound_ip_address() const
{
	boost::system::error_code errorCode;
	const auto endpoint = udpConnection.local_endpoint(errorCode);
	if (errorCode)
	{
		return "not available";
	}
	return endpoint.address().to_string();
}

udp::endpoint UdpConnections::get_local_endpoint() const
{
	auto subnet = settings->get_subnet();

#if !defined(_WIN32)
	// On POSIX, enumerate live interfaces via getifaddrs(3). The hostname-based
	// resolver below only returns whatever the resolver maps the hostname to
	// (e.g. 127.0.1.1 on stock Ubuntu), which never matches a LAN subnet.
	{
		struct ifaddrs *ifaList = nullptr;
		if (getifaddrs(&ifaList) == 0)
		{
			std::cout << "Available IP addresses (getifaddrs):" << std::endl;
			for (struct ifaddrs *ifa = ifaList; ifa != nullptr; ifa = ifa->ifa_next)
			{
				if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
				{
					continue;
				}
				auto *sin = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
				char buf[INET_ADDRSTRLEN] = { 0 };
				inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf));
				std::cout << "- " << ifa->ifa_name << " " << buf << std::endl;

				auto raw = ntohl(sin->sin_addr.s_addr);
				std::array<std::uint8_t, 4> octets = {
					static_cast<std::uint8_t>((raw >> 24) & 0xFF),
					static_cast<std::uint8_t>((raw >> 16) & 0xFF),
					static_cast<std::uint8_t>((raw >> 8) & 0xFF),
					static_cast<std::uint8_t>(raw & 0xFF),
				};
				if (std::equal(subnet.begin(), subnet.begin() + 3, octets.begin()))
				{
					auto addr = boost::asio::ip::make_address_v4(buf);
					std::cout << "Found local endpoint address " << buf << ", which matches the subnet " << settings->get_subnet_string() << std::endl;
					freeifaddrs(ifaList);
					return udp::endpoint(addr, 8888);
				}
			}
			freeifaddrs(ifaList);
		}
	}
#endif

	try
	{
		boost::asio::io_context io_context;
		boost::asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve(boost::asio::ip::host_name(), "");

		std::cout << "Available IP addresses (resolver):" << std::endl;
		for (const auto &endpoint : endpoints)
		{
			const auto &addr = endpoint.endpoint().address();
			std::cout << "- " << addr.to_string() << std::endl;

			if (addr.is_v4())
			{
				auto octets = addr.to_v4().to_bytes();
				if (std::equal(subnet.begin(), subnet.begin() + 3, octets.begin()))
				{
					std::cout << "Found local endpoint address " << addr.to_string() << ", which matches the subnet " << settings->get_subnet_string() << std::endl;
					return udp::endpoint(addr, 8888);
				}
			}
		}
	}
	catch (const std::exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
	}

	std::cout << "No suitable IP address found that matches the subnet " << settings->get_subnet_string() << ", using loopback address.\n";
	return udp::endpoint(boost::asio::ip::address_v4::loopback(), 8888);
}

std::uint8_t UdpConnections::calculate_crc(std::span<std::uint8_t> data)
{
	int result = 0;
	for (uint8_t i = 0; i < data.size(); i++)
	{
		result += data[i];
	}
	return static_cast<std::uint8_t>(result & 0xFF);
}

namespace
{
	void log_parse_stop(const char *socketName, std::span<std::uint8_t> datagram, const aog_packet::ParseResult &result)
	{
		if (aog_packet::ParseStop::Complete == result.stop)
		{
			return;
		}

		const std::size_t dropped = datagram.size() - result.offset;
		if (aog_packet::ParseStop::UnknownStart == result.stop)
		{
			const std::uint16_t start = static_cast<std::uint16_t>((datagram[result.offset] << 8) |
			                                                       ((result.offset + 1 < datagram.size()) ? datagram[result.offset + 1] : 0));
			log() << "Unknown start of message: 0x" << std::hex << start << std::dec
			      << " (" << socketName << ", dropped " << dropped << " of " << datagram.size() << " bytes)" << std::endl;
		}
		else
		{
			log() << "Truncated AOG packet (" << socketName << ", dropped " << dropped << " of " << datagram.size() << " bytes)" << std::endl;
		}
	}
}

void UdpConnections::handle_incoming_packets()
{
	static std::array<std::uint8_t, MAX_PACKET_SIZE> rxBuffer;

	boost::system::error_code error_code;
	udp::endpoint sender_endpoint;
	size_t bytesReceived = udpConnection.receive_from(boost::asio::buffer(rxBuffer), sender_endpoint, 0, error_code);

	if (error_code == boost::asio::error::would_block)
	{
		// No data available
	}
	else if (!error_code)
	{
		std::span<std::uint8_t> datagram(rxBuffer.data(), bytesReceived);
		auto result = aog_packet::parse_datagram(datagram, [this](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
			if (packetCallback)
			{
				packetCallback(src, pgn, data);
			}
		});
		log_parse_stop("AOG", datagram, result);
	}
	else
	{
		log() << "Error while receiving data: " << error_code.message() << std::endl;
	}
}

void UdpConnections::handle_address_detection()
{
	static std::array<std::uint8_t, MAX_PACKET_SIZE> rxBuffer;

	boost::system::error_code error_code;
	udp::endpoint sender_endpoint;
	size_t bytesReceived = udpConnectionAddressDetection.receive_from(boost::asio::buffer(rxBuffer), sender_endpoint, 0, error_code);

	if (error_code == boost::asio::error::would_block)
	{
		// No data available
	}
	else if (!error_code)
	{
		std::span<std::uint8_t> datagram(rxBuffer.data(), bytesReceived);
		auto result = aog_packet::parse_datagram(datagram, [this](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
			// 127 is source AgIO, 201 is subnet; data is 0xC9 0xC9 IP0 IP1 IP2
			if ((0x7F != src) || (0xC9 != pgn) || (5 != data.size()) || (0xC9 != data[0]) || (0xC9 != data[1]))
			{
				return;
			}

			settings->set_subnet({ data[2], data[3], data[4] });

			std::cout << "Subnet from AOG: ";
			std::cout << int(settings->get_subnet()[0]) << ".";
			std::cout << int(settings->get_subnet()[1]) << ".";
			std::cout << int(settings->get_subnet()[2]);
			std::cout << " rebinding UPD connection " << std::endl;
			udpConnection.close();
			udpConnection.open(udp::v4());
			udpConnection.set_option(boost::asio::socket_base::broadcast(true));
			udpConnection.set_option(boost::asio::socket_base::reuse_address(true));
			udpConnection.non_blocking(true);
			udpConnection.bind(get_local_endpoint());
		});
		log_parse_stop("address detection", datagram, result);
	}
	else
	{
		log() << "Error while receiving data: " << error_code.message() << std::endl;
	}
}

bool UdpConnections::send(std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data)
{
	std::array<std::uint8_t, 512> txBuffer;
	std::size_t index = 0;

	txBuffer[index++] = PACKET_START >> 8;
	txBuffer[index++] = PACKET_START & 0xFF;
	txBuffer[index++] = src;
	txBuffer[index++] = pgn;
	txBuffer[index++] = data.size();
	std::copy(data.begin(), data.end(), txBuffer.begin() + index);
	index += data.size();

	txBuffer[index] = calculate_crc({ txBuffer.data() + 2, index - 2 });

	auto subnet = settings->get_subnet();
	boost::asio::ip::address_v4 broadcast_address = boost::asio::ip::make_address_v4(std::to_string(subnet[0]) + "." +
	                                                                                 std::to_string(subnet[1]) + "." +
	                                                                                 std::to_string(subnet[2]) + ".255");

	udp::endpoint broadcast_endpoint(broadcast_address, 9999);
	try
	{
		udpConnection.send_to(boost::asio::buffer(txBuffer, index + 1), broadcast_endpoint);
	}
	catch (const boost::system::system_error &e)
	{
		// Probably wrong subnet, ignore
		return false;
	}
	return true;
}
