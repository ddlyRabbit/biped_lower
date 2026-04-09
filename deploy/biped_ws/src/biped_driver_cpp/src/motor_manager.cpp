/**
 * @file motor_manager.cpp
 * @brief Multi-bus motor management — calibration, clamping, soft-stop.
 */

#include "biped_driver_cpp/motor_manager.hpp"
#include "biped_driver_cpp/ankle_linkage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace biped_driver_cpp {

// ── URDF joint limits (fallback) ─────────────────────────────────

const std::unordered_map<std::string, std::pair<double, double>>&
BipedMotorManager::urdf_limits() {
    static const std::unordered_map<std::string, std::pair<double, double>> lim = {
        {"R_hip_pitch",  {-2.21657,  1.04720}},
        {"R_hip_roll",   {-2.26893,  0.20944}},
        {"R_hip_yaw",    {-1.57080,  1.57080}},
        {"R_knee",       { 0.00000,  2.70526}},
        {"L_hip_pitch",  {-1.04720,  2.21657}},
        {"L_hip_roll",   {-0.20944,  2.26893}},
        {"L_hip_yaw",    {-1.57080,  1.57080}},
        {"L_knee",       { 0.00000,  2.70526}},
    };
    return lim;
}

// ── Theoretical ankle motor limits (from URDF joint limits) ──────

static std::pair<double, double> ankle_motor_theoretical_limits(
    bool is_upper, int pitch_sign) {
    constexpr double pitch_min = -0.7267;
    constexpr double pitch_max =  0.42360;
    constexpr double roll_min  = -0.20180;
    constexpr double roll_max  =  0.20180;

    int sign = is_upper ? pitch_sign : -pitch_sign;
    double corners[4];
    int idx = 0;
    for (double p : {pitch_min, pitch_max}) {
        for (double r : {roll_min, roll_max}) {
            corners[idx++] = sign * PITCH_GAIN * p - ROLL_GAIN * r;
        }
    }
    double lo = *std::min_element(corners, corners + 4);
    double hi = *std::max_element(corners, corners + 4);
    return {lo, hi};
}

// ── Construction ─────────────────────────────────────────────────

BipedMotorManager::BipedMotorManager(const std::vector<JointConfig>& joints) {
    // Group by bus
    std::unordered_map<std::string, std::unordered_map<std::string, Motor>> bus_motors;
    std::unordered_map<std::string, std::unordered_map<std::string, CalibrationEntry>> bus_cal;

    for (auto& j : joints) {
        joints_[j.name] = j;
        joint_to_bus_[j.name] = j.can_bus;

        bus_motors[j.can_bus][j.name] = Motor{j.can_id, j.model};
        bus_cal[j.can_bus][j.name] = CalibrationEntry{j.direction, j.offset};
    }

    for (auto& [channel, motors] : bus_motors) {
        buses_[channel] = std::make_shared<RobstrideBus>(channel, motors, bus_cal[channel]);
    }
}

// ── YAML model string normalization ──────────────────────────────

static std::string normalize_model(const std::string& type_str) {
    // "RS04" → "rs-04", "RS02" → "rs-02"
    std::string digits;
    for (char c : type_str) {
        if (std::isdigit(c)) digits += c;
    }
    while (digits.size() < 2) digits = "0" + digits;
    return "rs-" + digits;
}

// ── Free function: build from YAML ───────────────────────────────
BipedMotorManager build_manager_from_yaml(
    const YAML::Node& robot_config,
    const YAML::Node& calibration) {

    std::vector<JointConfig> joints;

    for (auto bus_it = robot_config.begin(); bus_it != robot_config.end(); ++bus_it) {
        std::string bus_key = bus_it->first.as<std::string>();
        auto bus_cfg = bus_it->second;
        if (!bus_cfg.IsMap() || !bus_cfg["motors"]) continue;

        std::string iface = bus_cfg["interface"] ?
            bus_cfg["interface"].as<std::string>() : bus_key;

        auto motors_cfg = bus_cfg["motors"];
        for (auto m_it = motors_cfg.begin(); m_it != motors_cfg.end(); ++m_it) {
            std::string name = m_it->first.as<std::string>();
            auto mcfg = m_it->second;
            int id = mcfg["id"].as<int>();
            std::string model = normalize_model(mcfg["type"].as<std::string>());
            bool ankle = is_ankle_motor(name);

            double offset = 0.0;
            int direction = 1;
            double motor_cmd_lo = -12.57;
            double motor_cmd_hi = 12.57;

            auto& ulimits = BipedMotorManager::urdf_limits();
            auto urdf_it = ulimits.find(name);
            double urdf_lo = (urdf_it != ulimits.end()) ? urdf_it->second.first : -12.57;
            double urdf_hi = (urdf_it != ulimits.end()) ? urdf_it->second.second : 12.57;

            if (calibration && calibration[name]) {
                auto cal = calibration[name];
                double m_min = cal["motor_min"] ? cal["motor_min"].as<double>() : 0.0;
                double m_max = cal["motor_max"] ? cal["motor_max"].as<double>() : 0.0;
                direction = cal["direction"] ? cal["direction"].as<int>() : 1;

                if (direction == -1 && !ankle) {
                    offset = m_max + urdf_lo;
                } else {
                    offset = cal["offset"] ? cal["offset"].as<double>() : 0.0;
                }

                if (direction == -1) {
                    motor_cmd_lo = -(m_max - offset);
                    motor_cmd_hi = -(m_min - offset);
                } else {
                    motor_cmd_lo = m_min - offset;
                    motor_cmd_hi = m_max - offset;
                }
            } else {
                if (ankle) {
                    bool is_upper = is_ankle_top(name);
                    int pitch_sign = name[0] == 'L' ? -1 : 1;
                    auto [lo, hi] = ankle_motor_theoretical_limits(is_upper, pitch_sign);
                    motor_cmd_lo = lo;
                    motor_cmd_hi = hi;
                } else {
                    motor_cmd_lo = urdf_lo;
                    motor_cmd_hi = urdf_hi;
                }
            }

            double motor_softstop_lo = motor_cmd_lo + SOFTSTOP_BUFFER_RAD;
            double motor_softstop_hi = motor_cmd_hi - SOFTSTOP_BUFFER_RAD;

            joints.push_back(JointConfig{
                name, iface, id, model, offset, direction, ankle,
                motor_cmd_lo, motor_cmd_hi, motor_softstop_lo, motor_softstop_hi
            });
        }
    }

    return BipedMotorManager(joints);
}

// ── Connection ───────────────────────────────────────────────────

void BipedMotorManager::connect_all() {
    for (auto& [_, bus] : buses_) {
        if (!bus->is_connected()) bus->connect();
    }
}

void BipedMotorManager::disconnect_all() {
    for (auto& [_, bus] : buses_) {
        if (bus->is_connected()) bus->disconnect();
    }
}

void BipedMotorManager::enable_all() {
    for (auto& [_, bus] : buses_) {
        bus->enable_and_set_mit_all();
    }
}

void BipedMotorManager::disable_all(bool clear_fault) {
    for (auto& [_, bus] : buses_) {
        bus->disable_all(clear_fault);
    }
}

void BipedMotorManager::flush_all() {
    for (auto& [_, bus] : buses_) {
        bus->flush_rx();
    }
}

// ── Soft-stop torque ─────────────────────────────────────────────

double BipedMotorManager::compute_softstop_torque(
    const std::string& name, double actual_pos) const {
    auto it = joints_.find(name);
    if (it == joints_.end()) return 0.0;
    const auto& j = it->second;

    if (actual_pos < j.motor_softstop_lo) {
        double penetration = j.motor_softstop_lo - actual_pos;
        return SOFTSTOP_KP * std::min(penetration, SOFTSTOP_BUFFER_RAD);
    } else if (actual_pos > j.motor_softstop_hi) {
        double penetration = actual_pos - j.motor_softstop_hi;
        return -SOFTSTOP_KP * std::min(penetration, SOFTSTOP_BUFFER_RAD);
    }
    return 0.0;
}

// ── Motor commands ───────────────────────────────────────────────

RobstrideBus& BipedMotorManager::bus_for(const std::string& name) {
    return *buses_.at(joint_to_bus_.at(name));
}

void BipedMotorManager::send_mit_command(
    const std::string& name,
    double position, double kp, double kd,
    double velocity, double torque_ff,
    std::optional<double> actual_pos) {

    auto& j = joints_.at(name);

    // Hard clamp then soft-stop clamp
    position = std::max(j.motor_cmd_lo, std::min(j.motor_cmd_hi, position));
    position = std::max(j.motor_softstop_lo, std::min(j.motor_softstop_hi, position));

    if (actual_pos) {
        torque_ff += compute_softstop_torque(name, *actual_pos);
    }

    bus_for(name).write_operation_frame(name, position, kp, kd, velocity, torque_ff);
}

void BipedMotorManager::send_ankle_mit_command(
    const std::string& motor_name,
    double motor_position, double kp, double kd,
    double velocity, double torque_ff,
    std::optional<double> actual_pos) {

    auto& j = joints_.at(motor_name);

    motor_position = std::max(j.motor_cmd_lo, std::min(j.motor_cmd_hi, motor_position));
    motor_position = std::max(j.motor_softstop_lo, std::min(j.motor_softstop_hi, motor_position));

    if (actual_pos) {
        torque_ff += compute_softstop_torque(motor_name, *actual_pos);
    }

    bus_for(motor_name).write_operation_frame(motor_name, motor_position, kp, kd, velocity, torque_ff);
}

std::optional<MotorFeedback> BipedMotorManager::read_feedback(
    const std::string& name, double timeout) {
    return bus_for(name).receive_feedback(name, timeout);
}

// ── Ankle helpers ────────────────────────────────────────────────

std::optional<std::pair<std::string, std::string>>
BipedMotorManager::get_ankle_pair(const std::string& name) const {
    auto& pairs = ankle_pairs();
    auto it = pairs.find(name);
    if (it != pairs.end()) {
        return std::make_pair(it->first, it->second);
    }
    for (auto& [top, bot] : pairs) {
        if (bot == name) return std::make_pair(top, bot);
    }
    return std::nullopt;
}

}  // namespace biped_driver_cpp
