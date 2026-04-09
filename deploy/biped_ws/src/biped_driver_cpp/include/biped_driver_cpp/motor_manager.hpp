/**
 * @file motor_manager.hpp
 * @brief Multi-bus motor management for the biped robot.
 *
 * Routes commands to correct CAN bus, handles calibration limits,
 * soft-stop torque, and ankle pair management.
 */

#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "robstride_bus.hpp"

namespace biped_driver_cpp {

// ── Soft-stop config ─────────────────────────────────────────────

constexpr double SOFTSTOP_BUFFER_RAD = 2.0 * M_PI / 180.0;  // 0.0349 rad
constexpr double SOFTSTOP_KP = 20.0;  // Nm/rad restoring spring

// ── Ankle constants ──────────────────────────────────────────────

// Ankle pairs: top_motor → bottom_motor
inline const std::unordered_map<std::string, std::string>& ankle_pairs() {
    static const std::unordered_map<std::string, std::string> pairs = {
        {"L_foot_top", "L_foot_bottom"},
        {"R_foot_top", "R_foot_bottom"},
    };
    return pairs;
}

// Motor name → joint-space name
inline const std::unordered_map<std::string, std::string>& ankle_motor_to_joint() {
    static const std::unordered_map<std::string, std::string> map = {
        {"L_foot_top", "L_foot_pitch"}, {"L_foot_bottom", "L_foot_roll"},
        {"R_foot_top", "R_foot_pitch"}, {"R_foot_bottom", "R_foot_roll"},
    };
    return map;
}

inline bool is_ankle_motor(const std::string& name) {
    return ankle_motor_to_joint().count(name) > 0;
}

inline bool is_ankle_top(const std::string& name) {
    return ankle_pairs().count(name) > 0;
}

inline bool is_ankle_bottom(const std::string& name) {
    for (auto& [_, bot] : ankle_pairs()) {
        if (bot == name) return true;
    }
    return false;
}

// ── Joint config ─────────────────────────────────────────────────

struct JointConfig {
    std::string name;
    std::string can_bus;       // "can0", "can1"
    int can_id;
    std::string model;         // "rs-02", "rs-03", "rs-04"
    double offset = 0.0;
    int direction = 1;
    bool is_ankle = false;
    double motor_cmd_lo = -12.57;
    double motor_cmd_hi = 12.57;
    double motor_softstop_lo = -12.57;
    double motor_softstop_hi = 12.57;
};

// ── BipedMotorManager ────────────────────────────────────────────

class BipedMotorManager {
public:
    // URDF joint limits (fallback when no calibration)
    static const std::unordered_map<std::string, std::pair<double, double>>& urdf_limits();

    explicit BipedMotorManager(const std::vector<JointConfig>& joints);

    // Construction from YAML is via free function build_manager_from_yaml()

    // ── Connection ───────────────────────────────────────────────
    void connect_all();
    void disconnect_all();
    void enable_all();
    void disable_all(bool clear_fault = false);
    void flush_all();

    // ── Motor commands ───────────────────────────────────────────
    void send_mit_command(const std::string& name,
                          double position, double kp, double kd,
                          double velocity = 0.0, double torque_ff = 0.0,
                          std::optional<double> actual_pos = std::nullopt);

    void send_ankle_mit_command(const std::string& motor_name,
                                double motor_position, double kp, double kd,
                                double velocity = 0.0, double torque_ff = 0.0,
                                std::optional<double> actual_pos = std::nullopt);

    std::optional<MotorFeedback> read_feedback(const std::string& name,
                                                double timeout = 0.005);

    // ── Ankle helpers ────────────────────────────────────────────
    /// Returns (top, bottom) pair if name is part of an ankle, nullopt otherwise
    std::optional<std::pair<std::string, std::string>> get_ankle_pair(const std::string& name) const;

    // ── Accessors ────────────────────────────────────────────────
    const std::unordered_map<std::string, JointConfig>& joints() const { return joints_; }
    const std::unordered_map<std::string, std::shared_ptr<RobstrideBus>>& buses() const { return buses_; }

    double compute_softstop_torque(const std::string& name, double actual_pos) const;

private:
    RobstrideBus& bus_for(const std::string& name);

    std::unordered_map<std::string, JointConfig> joints_;
    std::unordered_map<std::string, std::shared_ptr<RobstrideBus>> buses_;
    std::unordered_map<std::string, std::string> joint_to_bus_;
};

}  // namespace biped_driver_cpp

// Forward declare YAML types (must be in global namespace)
namespace YAML { class Node; }

namespace biped_driver_cpp {

/// Build a BipedMotorManager from parsed YAML nodes.
BipedMotorManager build_manager_from_yaml(
    const YAML::Node& robot_config,
    const YAML::Node& calibration);

}  // namespace biped_driver_cpp
