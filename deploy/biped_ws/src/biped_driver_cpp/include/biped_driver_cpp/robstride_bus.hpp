/**
 * @file robstride_bus.hpp
 * @brief SocketCAN driver for RobStride motors — MIT-mode control.
 *
 * Direct Linux SocketCAN (AF_CAN), no external CAN library.
 * Per-model MIT scaling tables (RS02, RS03, RS04).
 * Calibration: offset + direction per motor.
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <optional>
#include <linux/can.h>

namespace biped_driver_cpp {

// ── Communication types (CAN ext ID bits 28-24) ──────────────────

namespace CommType {
constexpr uint32_t GET_DEVICE_ID       = 0;
constexpr uint32_t OPERATION_CONTROL   = 1;
constexpr uint32_t OPERATION_STATUS    = 2;
constexpr uint32_t ENABLE              = 3;
constexpr uint32_t DISABLE             = 4;
constexpr uint32_t SET_ZERO_POSITION   = 6;
constexpr uint32_t SET_DEVICE_ID       = 7;
constexpr uint32_t READ_PARAMETER      = 17;
constexpr uint32_t WRITE_PARAMETER     = 18;
constexpr uint32_t FAULT_REPORT        = 21;
}

// ── Parameter IDs ────────────────────────────────────────────────

namespace ParamID {
constexpr uint16_t MODE = 0x7005;
}

// ── Per-model MIT scaling ────────────────────────────────────────

struct MitScale {
    double position;  // ±rad
    double velocity;  // ±rad/s
    double torque;    // ±Nm
    double kp;        // 0..max
    double kd;        // 0..max
};

/// Get MIT scaling table for a model string ("rs-02", "rs-03", "rs-04", etc.)
const MitScale& get_mit_scale(const std::string& model);

// ── Motor / calibration config ───────────────────────────────────

struct Motor {
    int id;
    std::string model;  // "rs-02", "rs-03", "rs-04"
};

struct CalibrationEntry {
    int direction = 1;         // +1 normal, -1 inverted
    double homing_offset = 0.0;
};

// ── Feedback ─────────────────────────────────────────────────────

struct MotorFeedback {
    double position    = 0.0;  // rad (calibrated)
    double velocity    = 0.0;  // rad/s
    double torque      = 0.0;  // Nm
    double temperature = 0.0;  // °C
    int fault_code     = 0;
    int mode_status    = 0;    // 0=Reset, 1=Cal, 2=Run
};

// ── RobstrideBus ─────────────────────────────────────────────────

class RobstrideBus {
public:
    static constexpr uint16_t HOST_ID = 0xFD;

    RobstrideBus(const std::string& channel,
                 const std::unordered_map<std::string, Motor>& motors,
                 const std::unordered_map<std::string, CalibrationEntry>& calibration);
    ~RobstrideBus();

    // No copy
    RobstrideBus(const RobstrideBus&) = delete;
    RobstrideBus& operator=(const RobstrideBus&) = delete;

    void connect();
    void disconnect();
    bool is_connected() const { return socket_fd_ >= 0; }

    // ── Motor lifecycle ──────────────────────────────────────────
    std::optional<MotorFeedback> enable(const std::string& name);
    std::optional<MotorFeedback> disable(const std::string& name, bool clear_fault = false);
    void set_mode(const std::string& name, int mode);
    void set_zero_position(const std::string& name);

    // ── MIT frame I/O ────────────────────────────────────────────
    void write_operation_frame(const std::string& name,
                               double position, double kp, double kd,
                               double velocity = 0.0, double torque = 0.0);

    std::optional<MotorFeedback> receive_feedback(const std::string& name,
                                                       double timeout_sec = 0.01);

    // ── Bulk operations ──────────────────────────────────────────
    void enable_all();
    void disable_all(bool clear_fault = false);
    void enable_and_set_mit_all();
    int flush_rx(double timeout_sec = 0.005);

    // ── Accessors ────────────────────────────────────────────────
    const std::unordered_map<std::string, Motor>& motors() const { return motors_; }

private:
    void transmit(uint32_t comm_type, uint16_t extra_data,
                  uint8_t device_id, const uint8_t* data = nullptr, uint8_t dlc = 8);

    struct RxFrame {
        uint32_t comm_type;
        uint16_t extra_data;
        uint8_t host_id;
        uint8_t data[8];
    };
    std::optional<RxFrame> receive(double timeout_sec);

    std::optional<MotorFeedback> receive_feedback(const std::string& name,
                                                   double timeout_sec = 0.01);

    std::optional<float> read_parameter_float(const std::string& name, uint16_t param_id);

    std::string channel_;
    int socket_fd_ = -1;
    std::unordered_map<std::string, Motor> motors_;
    std::unordered_map<std::string, CalibrationEntry> calibration_;
};

}  // namespace biped_driver_cpp
