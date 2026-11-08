#include "isobridge_can_plugin.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <thread>

namespace
{
constexpr std::uint16_t PORT = 9200;
constexpr std::uint8_t MAGIC_A = 'A', MAGIC_C = 'C', VERSION = 1;
constexpr std::uint8_t TYPE_DATA = 1, TYPE_HELLO = 2, TYPE_HELLO_ACK = 3, TYPE_STATS = 4, TYPE_BYE = 5;
constexpr std::uint8_t MAX_FRAMES = 88;
constexpr std::size_t HEADER_SIZE = 8, FRAME_SIZE = 16, MAX_PACKET = HEADER_SIZE + MAX_FRAMES * FRAME_SIZE;
constexpr std::uint32_t EXTENDED_ID = 0x80000000UL;

void put16(std::uint8_t *p, std::uint16_t v) { p[0] = static_cast<std::uint8_t>(v); p[1] = static_cast<std::uint8_t>(v >> 8); }
void put32(std::uint8_t *p, std::uint32_t v) { for (int i=0;i<4;i++) p[i] = static_cast<std::uint8_t>(v >> (8*i)); }
std::uint16_t get16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8)); }
std::uint32_t get32(const std::uint8_t *p) { return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1])<<8) | (static_cast<std::uint32_t>(p[2])<<16) | (static_cast<std::uint32_t>(p[3])<<24); }
}

ISOBridgeCANPlugin::ISOBridgeCANPlugin(const std::string &gatewayAddress) : socket_(io_)
{
    boost::system::error_code ec;
    auto address = boost::asio::ip::make_address_v4(gatewayAddress, ec);
    if (ec) { set_error("Invalid ISOBridge IPv4 address: " + gatewayAddress); return; }
    gateway_ = udp::endpoint(address, PORT);
    addressValid_ = true;
}

ISOBridgeCANPlugin::~ISOBridgeCANPlugin() { close(); }
std::string ISOBridgeCANPlugin::get_name() const { return "AgOpenGPS ISOBridge UDP"; }
bool ISOBridgeCANPlugin::get_is_valid() const { return valid_.load(); }
std::string ISOBridgeCANPlugin::get_last_error() const { std::lock_guard<std::mutex> lock(mutex_); return error_; }
void ISOBridgeCANPlugin::set_error(const std::string &message) { error_ = message; }

bool ISOBridgeCANPlugin::send_packet_locked(std::uint8_t type, std::uint8_t count, const std::uint8_t *body, std::size_t bodySize)
{
    std::array<std::uint8_t, MAX_PACKET> packet{};
    if (HEADER_SIZE + bodySize > packet.size()) return false;
    packet[0] = MAGIC_A; packet[1] = MAGIC_C; packet[2] = VERSION; packet[3] = type;
    put16(&packet[4], txSequence_++); packet[6] = count; packet[7] = 0;
    if (bodySize && body) std::memcpy(packet.data() + HEADER_SIZE, body, bodySize);
    boost::system::error_code ec;
    const auto sent = socket_.send_to(boost::asio::buffer(packet.data(), HEADER_SIZE + bodySize), gateway_, 0, ec);
    if (ec || sent != HEADER_SIZE + bodySize) { set_error("ISOBridge UDP send failed: " + ec.message()); return false; }
    return true;
}

bool ISOBridgeCANPlugin::send_hello_locked()
{
    // HELLO body v1: client kind, options, two reserved bytes, 16-byte client name.
    // kind=1 identifies a PC-side application; opts=0 disables sender echo.
    std::array<std::uint8_t, 20> body{};
    body[0] = 1; body[1] = 0;
    constexpr char name[] = "AOG-TaskController";
    std::memcpy(body.data() + 4, name, std::min<std::size_t>(16, sizeof(name)-1));
    const bool ok = send_packet_locked(TYPE_HELLO, 0, body.data(), body.size());
    if (ok) lastHello_ = std::chrono::steady_clock::now();
    return ok;
}

void ISOBridgeCANPlugin::open()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (valid_) return;
    if (!addressValid_) return;
    boost::system::error_code ec;
    socket_.open(udp::v4(), ec);
    if (ec) { set_error("Cannot open UDP socket: " + ec.message()); return; }
    socket_.bind(udp::endpoint(udp::v4(), 0), ec);
    if (ec) { set_error("Cannot bind UDP socket: " + ec.message()); socket_.close(); return; }
    socket_.non_blocking(true, ec);
    if (ec) { set_error("Cannot set UDP socket non-blocking: " + ec.message()); socket_.close(); return; }

    set_error("Waiting for ISOBridge HELLO_ACK...");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    std::array<std::uint8_t, 128> buffer{};
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (!send_hello_locked()) break;
        // Wait briefly for the Teensy's response before retrying HELLO.
        const auto retryAt = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        while (std::chrono::steady_clock::now() < retryAt)
        {
            udp::endpoint from;
            const auto n = socket_.receive_from(boost::asio::buffer(buffer), from, 0, ec);
            if (!ec && from.address() == gateway_.address() && from.port() == PORT && n >= HEADER_SIZE + 28 &&
                buffer[0] == MAGIC_A && buffer[1] == MAGIC_C && buffer[2] == VERSION && buffer[3] == TYPE_HELLO_ACK)
            {
                clientId_ = buffer[HEADER_SIZE];
                if (clientId_ != 0xFF)
                {
                    valid_ = true; error_.clear();
                    rxExpected_ = static_cast<std::uint16_t>(get16(&buffer[4]) + 1); rxSequenceStarted_ = true;
                    return;
                }
                set_error("ISOBridge refused client registration: client table full (4 clients).");
                socket_.close(); return;
            }
            if (ec != boost::asio::error::would_block && ec != boost::asio::error::try_again && ec)
            { set_error("ISOBridge UDP receive failed: " + ec.message()); socket_.close(); return; }
            ec.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    if (socket_.is_open()) socket_.close();
    set_error("No HELLO_ACK from ISOBridge at " + gateway_.address().to_string() + ":9200. Check Teensy, Ethernet and UDP firewall.");
}

void ISOBridgeCANPlugin::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (socket_.is_open() && valid_)
    {
        std::array<std::uint8_t, 0> empty{};
        send_packet_locked(TYPE_BYE, 0, empty.data(), 0);
    }
    valid_ = false; clientId_ = 0xFF; receivedFrames_.clear();
    boost::system::error_code ec; if (socket_.is_open()) socket_.close(ec);
}

bool ISOBridgeCANPlugin::receive_one_locked()
{
    std::array<std::uint8_t, MAX_PACKET> packet{};
    udp::endpoint from;
    boost::system::error_code ec;
    const auto n = socket_.receive_from(boost::asio::buffer(packet), from, 0, ec);
    if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) return false;
    if (ec) { set_error("ISOBridge UDP receive error: " + ec.message()); valid_ = false; return false; }
    if (from.address() != gateway_.address() || from.port() != PORT || n < HEADER_SIZE) return true;
    if (packet[0] != MAGIC_A || packet[1] != MAGIC_C || packet[2] != VERSION) return true;
    const auto type = packet[3];
    const auto seq = get16(&packet[4]);
    const auto count = packet[6];
    if (rxSequenceStarted_) rxExpected_ = static_cast<std::uint16_t>(seq + 1);
    else { rxExpected_ = static_cast<std::uint16_t>(seq + 1); rxSequenceStarted_ = true; }

    if (type == TYPE_HELLO_ACK && n >= HEADER_SIZE + 28)
    {
        clientId_ = packet[HEADER_SIZE];
        if (clientId_ == 0xFF) { set_error("ISOBridge client table is full."); valid_ = false; }
    }
    else if (type == TYPE_DATA)
    {
        if (n != HEADER_SIZE + static_cast<std::size_t>(count) * FRAME_SIZE || count > MAX_FRAMES) return true;
        for (std::uint8_t i=0; i<count; ++i)
        {
            const auto *p = packet.data() + HEADER_SIZE + static_cast<std::size_t>(i) * FRAME_SIZE;
            const auto ident = get32(p);
            const auto dlc = p[4];
            if (dlc > 8) continue;
            isobus::CANMessageFrame frame{};
            frame.timestamp_us = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
            frame.identifier = ident & ~EXTENDED_ID;
            frame.isExtendedFrame = (ident & EXTENDED_ID) != 0;
            frame.channel = 0; frame.dataLength = dlc;
            std::memcpy(frame.data, p + 8, dlc);
            receivedFrames_.push_back(frame);
        }
    }
    else if (type == TYPE_STATS && n >= HEADER_SIZE + 40)
    {
        // Statistics are currently informational; keepalive/sequence is still tracked.
    }
    return true;
}

bool ISOBridgeCANPlugin::read_frame(isobus::CANMessageFrame &canFrame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_) return false;
    if (std::chrono::steady_clock::now() - lastHello_ >= std::chrono::seconds(1)) send_hello_locked();
    if (receivedFrames_.empty()) receive_one_locked();
    if (receivedFrames_.empty()) return false;
    canFrame = receivedFrames_.front(); receivedFrames_.pop_front();
    return true;
}

bool ISOBridgeCANPlugin::write_frame(const isobus::CANMessageFrame &canFrame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_) return false;
    const auto now = std::chrono::steady_clock::now();
    if (now - lastHello_ >= std::chrono::seconds(1)) send_hello_locked();
    std::array<std::uint8_t, FRAME_SIZE> body{};
    const auto ident = (canFrame.identifier & 0x1FFFFFFFUL) | (canFrame.isExtendedFrame ? EXTENDED_ID : 0);
    put32(body.data(), ident); body[4] = std::min<std::uint8_t>(canFrame.dataLength, 8); body[5] = 0; put16(&body[6], 0);
    std::memcpy(body.data() + 8, canFrame.data, body[4]);
    return send_packet_locked(TYPE_DATA, 1, body.data(), body.size());
}
