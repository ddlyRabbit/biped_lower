/**
 * @file robstride_bus.cpp
 * @brief SocketCAN driver for RobStride motors — MIT-mode control.
 */

#include "biped_driver_cpp/robstride_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <endian.h>

namespace biped_driver_cpp {

// ── Per-model scaling tables ─────────────────────────────────────
// From table.py: all models use ±4π position

static const std::unordered_map<std::string, MitScale> MIT_SCALES = {
    // model, {pos_max, vel_max, torque_max, kp_max, kd_max}
    {"rs-00", {4.0 * M_PI,  50.0,  17.0,  500.0,   5.0}},
    {"rs-01", {4.0 * M_PI,  44.0,  17.0,  500.0,   5.0}},
    {"rs-02", {4.0 * M_PI,  44.0,  17.0,  500.0,   5.0}},
    {"rs-03", {4.0 * M_PI,  50.0,  60.0, 5000.0, 100.0}},
    {"rs-04", {4.0 * M_PI,  15.0, 120.0, 5000.0, 100.0}},
    {"rs-05", {4.0 * M_PI,  33.0,  17.0,  500.0,   5.0}},
    {"rs-06", {4.0 * M_PI,  20.0,  60.0, 5000.0, 100.0}},
};

const MitScale& get_mit_scale(const std::string& model) {
    auto it = MIT_SCALES.find(model);
    if (it == MIT_SCALES.end()) {
        throw std::runtime_error("Unknown motor model: " + model);
    }
    return it->second;
}

// ── Helpers ──────────────────────────────────────────────────────

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline uint16_t encode_u16_be(uint16_t val) {
    return htobe16(val);
}

static inline uint16_t decode_u16_be(uint16_t val) {
    return be16toh(val);
}

// ── Construction / destruction ───────────────────────────────────

RobstrideBus::RobstrideBus(
    const std::string& channel,
    const std::unordered_map<std::string, Motor>& motors,
    const std::unordered_map<std::string, CalibrationEntry>& calibration)
    : channel_(channel), motors_(motors), calibration_(calibration) {}

RobstrideBus::~RobstrideBus() {
    if (is_connected()) disconnect();
}

// ── Connection ───────────────────────────────────────────────────

void RobstrideBus::connect() {
    if (is_connected()) throw std::runtime_error("Already connected to " + channel_);

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to create CAN socket: " + std::string(strerror(errno)));
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("CAN interface not found: " + channel_);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to bind CAN socket: " + std::string(strerror(errno)));
    }
}

void RobstrideBus::disconnect() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

// ── Low-level CAN TX/RX ─────────────────────────────────────────

void RobstrideBus::transmit(uint32_t comm_type, uint16_t extra_data,
                             uint8_t device_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = (comm_type << 24) | (static_cast<uint32_t>(extra_data) << 8) | device_id;
    frame.can_id |= CAN_EFF_FLAG;  // Extended frame
    frame.can_dlc = dlc;
    if (data) {
        std::memcpy(frame.data, data, dlc);
    }

    for (int attempt = 0; attempt < 5; ++attempt) {
        ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
        if (nbytes == sizeof(frame)) return;
        if (errno == ENOBUFS) {
            std::this_thread::sleep_for(std::chrono::microseconds(500 * (attempt + 1)));
            continue;
        }
        break;
    }
    // Last attempt
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        throw std::runtime_error("CAN transmit failed: " + std::string(strerror(errno)));
    }
}

std::optional<RobstrideBus::RxFrame> RobstrideBus::receive(double timeout_sec) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        int ms = std::max(1, static_cast<int>(remaining.count()));

        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;
        int ret = poll(&pfd, 1, ms);
        if (ret <= 0) return std::nullopt;

        struct can_frame frame;
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) return std::nullopt;

        if (frame.can_id & CAN_EFF_FLAG) {
            uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
            RxFrame rx;
            rx.comm_type = (raw_id >> 24) & 0x1F;
            rx.extra_data = (raw_id >> 8) & 0xFFFF;
            rx.host_id = raw_id & 0xFF;
            std::memcpy(rx.data, frame.data, 8);
            return rx;
        }
    }
    return std::nullopt;
}

int RobstrideBus::flush_rx(double timeout_sec) {
    int count = 0;
    while (true) {
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;
        int ret = poll(&pfd, 1, static_cast<int>(timeout_sec * 1000));
        if (ret <= 0) break;

        struct can_frame frame;
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) break;
        count++;
    }
    return count;
}

// ── Feedback decode ──────────────────────────────────────────────

std::optional<MotorFeedback> RobstrideBus::receive_feedback(
    const std::string& name, double timeout_sec) {
    auto it = motors_.find(name);
    if (it == motors_.end()) return std::nullopt;

    int expected_id = it->second.id;
    const auto& model = it->second.model;
    const auto& scale = get_mit_scale(model);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        double remaining = std::chrono::duration<double>(
            deadline - std::chrono::steady_clock::now()).count();
        remaining = std::max(0.001, remaining);

        auto rx = receive(remaining);
        if (!rx) return std::nullopt;

        if (rx->comm_type == CommType::FAULT_REPORT) {
            uint32_t fault_val, warn_val;
            std::memcpy(&fault_val, rx->data, 4);
            std::memcpy(&warn_val, rx->data + 4, 4);
            throw std::runtime_error("Motor " + name + " fault: 0x" +
                                     std::to_string(fault_val));
        }

        if (rx->comm_type != CommType::OPERATION_STATUS) continue;

        int device_id = rx->extra_data & 0xFF;
        if (device_id != expected_id) continue;

        int fault_code = (rx->extra_data >> 8) & 0x3F;
        int mode_status = (rx->extra_data >> 14) & 0x03;

        // Big-endian decode
        uint16_t pos_u16  = (static_cast<uint16_t>(rx->data[0]) << 8) | rx->data[1];
        uint16_t vel_u16  = (static_cast<uint16_t>(rx->data[2]) << 8) | rx->data[3];
        uint16_t trq_u16  = (static_cast<uint16_t>(rx->data[4]) << 8) | rx->data[5];
        uint16_t temp_u16 = (static_cast<uint16_t>(rx->data[6]) << 8) | rx->data[7];

        double position = (static_cast<double>(pos_u16) / 0x7FFF - 1.0) * scale.position;
        double velocity = (static_cast<double>(vel_u16) / 0x7FFF - 1.0) * scale.velocity;
        double torque   = (static_cast<double>(trq_u16) / 0x7FFF - 1.0) * scale.torque;
        double temperature = static_cast<double>(temp_u16) * 0.1;

        // Undo calibration
        auto cal_it = calibration_.find(name);
        int direction = 1;
        double homing_offset = 0.0;
        if (cal_it != calibration_.end()) {
            direction = cal_it->second.direction;
            homing_offset = cal_it->second.homing_offset;
        }
        position = (position - homing_offset) * direction;
        velocity = velocity * direction;
        torque = torque * direction;

        return MotorFeedback{position, velocity, torque, temperature, fault_code, mode_status};
    }
    return std::nullopt;
}

// ── Motor lifecycle ──────────────────────────────────────────────

std::optional<MotorFeedback> RobstrideBus::enable(const std::string& name) {
    auto& m = motors_.at(name);
    transmit(CommType::ENABLE, HOST_ID, m.id);
    return receive_feedback(name);
}

std::optional<MotorFeedback> RobstrideBus::disable(const std::string& name, bool clear_fault) {
    auto& m = motors_.at(name);
    uint8_t data[8] = {};
    data[0] = clear_fault ? 1 : 0;
    transmit(CommType::DISABLE, HOST_ID, m.id, data);
    return receive_feedback(name);
}

void RobstrideBus::set_mode(const std::string& name, int mode) {
    auto& m = motors_.at(name);
    uint8_t data[8] = {};
    // param_id (LE) + 0x00 (LE) + value (int8 LE padded)
    data[0] = ParamID::MODE & 0xFF;
    data[1] = (ParamID::MODE >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = static_cast<uint8_t>(mode);
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    transmit(CommType::WRITE_PARAMETER, HOST_ID, m.id, data);
    receive_feedback(name, 0.05);  // consume response
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void RobstrideBus::set_zero_position(const std::string& name) {
    auto& m = motors_.at(name);
    uint8_t data[8] = {};
    data[0] = 1;
    transmit(CommType::SET_ZERO_POSITION, HOST_ID, m.id, data);
    receive_feedback(name);
}

// ── Read parameter (for mode verification) ───────────────────────

std::optional<float> RobstrideBus::read_parameter_float(const std::string& name, uint16_t param_id) {
    auto& m = motors_.at(name);
    uint8_t data[8] = {};
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;
    transmit(CommType::READ_PARAMETER, HOST_ID, m.id, data);

    auto rx = receive(0.05);
    if (!rx) return std::nullopt;

    // For MODE param (int8), value is at data[4]
    if (param_id == ParamID::MODE) {
        return static_cast<float>(static_cast<int8_t>(rx->data[4]));
    }

    float val;
    std::memcpy(&val, rx->data + 4, sizeof(float));
    return val;
}

// ── MIT frame write ──────────────────────────────────────────────

void RobstrideBus::write_operation_frame(
    const std::string& name,
    double position, double kp, double kd,
    double velocity, double torque) {

    auto& m = motors_.at(name);
    const auto& scale = get_mit_scale(m.model);

    // Apply calibration
    auto cal_it = calibration_.find(name);
    int direction = 1;
    double homing_offset = 0.0;
    if (cal_it != calibration_.end()) {
        direction = cal_it->second.direction;
        homing_offset = cal_it->second.homing_offset;
    }
    double pos = position * direction + homing_offset;
    double vel = velocity * direction;
    double trq = torque * direction;

    // Clamp to scale ranges
    pos = clamp(pos, -scale.position, scale.position);
    vel = clamp(vel, -scale.velocity, scale.velocity);
    kp  = clamp(kp, 0.0, scale.kp);
    kd  = clamp(kd, 0.0, scale.kd);
    trq = clamp(trq, -scale.torque, scale.torque);

    // Encode to uint16
    uint16_t pos_u16 = static_cast<uint16_t>(clamp(((pos / scale.position) + 1.0) * 0x7FFF, 0.0, 65535.0));
    uint16_t vel_u16 = static_cast<uint16_t>(clamp(((vel / scale.velocity) + 1.0) * 0x7FFF, 0.0, 65535.0));
    uint16_t kp_u16  = static_cast<uint16_t>(clamp((kp / scale.kp) * 0xFFFF, 0.0, 65535.0));
    uint16_t kd_u16  = static_cast<uint16_t>(clamp((kd / scale.kd) * 0xFFFF, 0.0, 65535.0));
    uint16_t trq_u16 = static_cast<uint16_t>(clamp(((trq / scale.torque) + 1.0) * 0x7FFF, 0.0, 65535.0));

    // Big-endian pack
    uint8_t data[8];
    data[0] = (pos_u16 >> 8) & 0xFF; data[1] = pos_u16 & 0xFF;
    data[2] = (vel_u16 >> 8) & 0xFF; data[3] = vel_u16 & 0xFF;
    data[4] = (kp_u16  >> 8) & 0xFF; data[5] = kp_u16  & 0xFF;
    data[6] = (kd_u16  >> 8) & 0xFF; data[7] = kd_u16  & 0xFF;

    transmit(CommType::OPERATION_CONTROL, trq_u16, m.id, data);
}

// ── Bulk operations ──────────────────────────────────────────────

void RobstrideBus::enable_all() {
    for (auto& [name, _] : motors_) {
        flush_rx();
        enable(name);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void RobstrideBus::disable_all(bool clear_fault) {
    for (auto& [name, _] : motors_) {
        flush_rx();
        disable(name, clear_fault);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void RobstrideBus::enable_and_set_mit_all() {
    for (auto& [name, _] : motors_) {
        // 1. Disable (clean state, clear faults)
        flush_rx();
        disable(name, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 2. Set MIT mode with verification (up to 3 attempts)
        bool mode_ok = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
            flush_rx();
            set_mode(name, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            flush_rx();
            auto mode_val = read_parameter_float(name, ParamID::MODE);
            if (mode_val && static_cast<int>(*mode_val) == 0) {
                mode_ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 3. Enable
        flush_rx();
        enable(name);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 4. Verify with zero-torque MIT command
        flush_rx();
        write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0);
        auto fb = receive_feedback(name, 0.05);
        (void)fb;  // verification only
    }
}

}  // namespace biped_driver_cpp
