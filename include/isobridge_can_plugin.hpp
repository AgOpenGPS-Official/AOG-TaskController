#pragma once

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include <boost/asio.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

/// UDP CAN driver for the AgOpenGPS Teensy ISOBridge protocol (AC v1, UDP/9200).
class ISOBridgeCANPlugin final : public isobus::CANHardwarePlugin
{
public:
    explicit ISOBridgeCANPlugin(const std::string &gatewayAddress);
    ~ISOBridgeCANPlugin() override;

    std::string get_name() const override;
    bool get_is_valid() const override;
    void close() override;
    void open() override;
    bool read_frame(isobus::CANMessageFrame &canFrame) override;
    bool write_frame(const isobus::CANMessageFrame &canFrame) override;

    std::string get_last_error() const override;

private:
    using udp = boost::asio::ip::udp;
    bool send_hello_locked();
    bool send_packet_locked(std::uint8_t type, std::uint8_t count, const std::uint8_t *body, std::size_t bodySize);
    bool receive_one_locked();
    void set_error(const std::string &message);

    boost::asio::io_context io_;
    udp::socket socket_;
    udp::endpoint gateway_;
    udp::endpoint sender_;
    mutable std::mutex mutex_;
    std::deque<isobus::CANMessageFrame> receivedFrames_;
    std::atomic_bool valid_{false};
    std::uint16_t txSequence_ = 0;
    std::uint16_t rxExpected_ = 0;
    bool rxSequenceStarted_ = false;
    bool addressValid_ = false;
    std::uint8_t clientId_ = 0xFF;
    std::chrono::steady_clock::time_point lastHello_{};
    std::string error_;
};
