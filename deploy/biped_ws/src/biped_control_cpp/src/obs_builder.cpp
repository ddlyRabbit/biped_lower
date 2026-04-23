#include "biped_control_cpp/obs_builder.hpp"
#include <cmath>

namespace biped_control_cpp {

const std::vector<std::string> ISAAC_JOINT_ORDER = {
    "L_hip_pitch", "R_hip_pitch",
    "L_hip_roll", "R_hip_roll",
    "L_hip_yaw", "R_hip_yaw",
    "L_knee", "R_knee",
    "L_foot_pitch", "R_foot_pitch",
    "L_foot_roll", "R_foot_roll",
};

const std::vector<std::string> JOINT_ORDER = ISAAC_JOINT_ORDER;

const std::vector<std::string> HIP_POS_ORDER = {
    "L_hip_roll", "R_hip_roll", "L_hip_yaw", "R_hip_yaw", "L_hip_pitch", "R_hip_pitch"
};

const std::vector<std::string> KNEE_POS_ORDER = {"L_knee", "R_knee"};
const std::vector<std::string> FOOT_PITCH_ORDER = {"L_foot_pitch", "R_foot_pitch"};
const std::vector<std::string> FOOT_ROLL_ORDER = {"L_foot_roll", "R_foot_roll"};

const std::vector<std::string> ACTION_ORDER = {
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch",
    "R_knee", "R_foot_pitch", "R_foot_roll",
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch",
    "L_knee", "L_foot_pitch", "L_foot_roll",
};

const std::vector<std::string> CSV_JOINT_ORDER = {
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_foot_pitch", "L_foot_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_foot_pitch", "R_foot_roll",
};

// Empty maps, populated at startup by `load_control_params()`
std::unordered_map<std::string, double> DEFAULT_POSITIONS;
std::unordered_map<std::string, std::pair<double, double>> DEFAULT_GAINS;
std::unordered_map<std::string, std::pair<double, double>> JOINT_LIMITS;

void load_control_params(const std::string& path_in) {
    std::string path = path_in.empty() ? "/home/roy/biped_lower/deploy/biped_ws/src/biped_bringup/config/control_params.yaml" : path_in;
    
    YAML::Node config;
    try {
        config = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Fatal: Could not load control_params.yaml from " + path + " - " + e.what());
    }

    if (!config["control_params"] || !config["control_params"]["joints"]) {
        throw std::runtime_error("Fatal: Invalid format in control_params.yaml");
    }

    auto joints = config["control_params"]["joints"];
    for (YAML::const_iterator it = joints.begin(); it != joints.end(); ++it) {
        std::string name = it->first.as<std::string>();
        auto params = it->second;

        DEFAULT_POSITIONS[name] = params["default_pos"].as<double>();
        DEFAULT_GAINS[name] = {params["kp"].as<double>(), params["kd"].as<double>()};
        JOINT_LIMITS[name] = {params["limit_min"].as<double>(), params["limit_max"].as<double>()};
    }
}

ObsBuilder::ObsBuilder() {
    last_action_.fill(0.0f);
}

std::array<float, 45> ObsBuilder::build(
    const std::array<float, 3>& gyro,
    const std::array<float, 3>& gravity,
    const std::array<float, 3>& cmd_vel,
    const std::unordered_map<std::string, double>& joint_positions,
    const std::unordered_map<std::string, double>& joint_velocities
) const {
    std::array<float, 45> obs = {0};

    // [0-2] base_ang_vel
    obs[0] = gyro[0];
    obs[1] = gyro[1];
    obs[2] = gyro[2];

    // [3-5] projected_gravity
    // The gravity vector from the IMU node is already a normalized unit vector
    // pointing downwards in the body frame, perfectly matching Isaac Sim.
    obs[3] = gravity[0];
    obs[4] = gravity[1];
    obs[5] = gravity[2];

    // [6-8] velocity_commands
    obs[6] = cmd_vel[0];
    obs[7] = cmd_vel[1];
    obs[8] = cmd_vel[2];

    // [9-14] hip_pos (relative to default)
    for (size_t i = 0; i < HIP_POS_ORDER.size(); ++i) {
        auto it = joint_positions.find(HIP_POS_ORDER[i]);
        double pos = (it != joint_positions.end()) ? it->second : 0.0;
        obs[9 + i] = static_cast<float>(pos - DEFAULT_POSITIONS.at(HIP_POS_ORDER[i]));
    }

    // [15-16] knee_pos
    for (size_t i = 0; i < KNEE_POS_ORDER.size(); ++i) {
        auto it = joint_positions.find(KNEE_POS_ORDER[i]);
        double pos = (it != joint_positions.end()) ? it->second : 0.0;
        obs[15 + i] = static_cast<float>(pos - DEFAULT_POSITIONS.at(KNEE_POS_ORDER[i]));
    }

    // [17-18] foot_pitch_pos
    for (size_t i = 0; i < FOOT_PITCH_ORDER.size(); ++i) {
        auto it = joint_positions.find(FOOT_PITCH_ORDER[i]);
        double pos = (it != joint_positions.end()) ? it->second : 0.0;
        obs[17 + i] = static_cast<float>(pos - DEFAULT_POSITIONS.at(FOOT_PITCH_ORDER[i]));
    }

    // [19-20] foot_roll_pos
    for (size_t i = 0; i < FOOT_ROLL_ORDER.size(); ++i) {
        auto it = joint_positions.find(FOOT_ROLL_ORDER[i]);
        double pos = (it != joint_positions.end()) ? it->second : 0.0;
        obs[19 + i] = static_cast<float>(pos - DEFAULT_POSITIONS.at(FOOT_ROLL_ORDER[i]));
    }

    // [21-32] joint_vel (Isaac runtime order)
    for (size_t i = 0; i < ISAAC_JOINT_ORDER.size(); ++i) {
        auto it = joint_velocities.find(ISAAC_JOINT_ORDER[i]);
        double vel = (it != joint_velocities.end()) ? it->second : 0.0;
        obs[21 + i] = static_cast<float>(vel);
    }

    // [33-44] last_action
    for (size_t i = 0; i < 12; ++i) {
        obs[33 + i] = last_action_[i];
    }

    return obs;
}

void ObsBuilder::update_last_action(const std::array<float, 12>& action) {
    last_action_ = action;
}

std::unordered_map<std::string, double> ObsBuilder::action_to_positions(const std::array<float, 12>& action) {
    std::unordered_map<std::string, double> targets;
    for (size_t i = 0; i < ACTION_ORDER.size(); ++i) {
        const std::string& name = ACTION_ORDER[i];
        float scale = 0.5f;
        if (name == "R_foot_roll" || name == "L_foot_roll") {
            scale = 0.25f;
        }
        targets[name] = DEFAULT_POSITIONS.at(name) + action[i] * scale;
    }
    return targets;
}

} // namespace biped_control_cpp