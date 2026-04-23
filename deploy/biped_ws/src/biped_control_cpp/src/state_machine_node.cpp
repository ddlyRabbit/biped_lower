#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "biped_msgs/msg/mit_command.hpp"
#include "biped_msgs/msg/mit_command_array.hpp"
#include "biped_msgs/msg/robot_state.hpp"

#include <yaml-cpp/yaml.h>
#include "biped_control_cpp/obs_builder.hpp"

using namespace std::chrono_literals;
using namespace biped_control_cpp;

class StateMachineNodeCpp : public rclcpp::Node {
public:
    StateMachineNodeCpp() : Node("state_machine_node") {
        declare_parameter("stand_ramp_time", 2.0);
        declare_parameter("stand_gain_ramp_time", 1.0);
        declare_parameter("stand_stable_time", 2.0);
        declare_parameter("wiggle_config", "");
        declare_parameter("gain_scale", 1.0);
        declare_parameter("trajectory_file", "");

        ramp_time_ = get_parameter("stand_ramp_time").as_double();
        gain_ramp_time_ = get_parameter("stand_gain_ramp_time").as_double();
        stable_time_ = get_parameter("stand_stable_time").as_double();

        auto sensor_qos = rclcpp::QoS(1).best_effort();

        sub_safety_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/status", 10, std::bind(&StateMachineNodeCpp::safety_cb, this, std::placeholders::_1));
        sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", sensor_qos, std::bind(&StateMachineNodeCpp::joint_cb, this, std::placeholders::_1));
        sub_cmd_ = create_subscription<std_msgs::msg::String>(
            "/state_command", 10, std::bind(&StateMachineNodeCpp::command_cb, this, std::placeholders::_1));

        pub_state_ = create_publisher<std_msgs::msg::String>("/state_machine", 10);
        pub_robot_ = create_publisher<biped_msgs::msg::RobotState>("/robot_state", 10);
        pub_cmd_ = create_publisher<biped_msgs::msg::MITCommandArray>("/joint_commands", 10);
        pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_viz_js_ = create_publisher<sensor_msgs::msg::JointState>("/policy_viz_joints", 10);

        timer_control_ = create_wall_timer(20ms, std::bind(&StateMachineNodeCpp::loop_cb, this));
        timer_status_ = create_wall_timer(100ms, std::bind(&StateMachineNodeCpp::publish_state_cb, this));

        state_ = "IDLE";
        state_start_time_ = now().seconds();
        
        for (const auto& name : JOINT_ORDER) {
            current_positions_[name] = 0.0;
        }

        RCLCPP_INFO(get_logger(), "State machine started (C++) — state: %s", state_.c_str());
    }

private:
    double ramp_time_, gain_ramp_time_, stable_time_;
    
    std::string state_;
    double state_start_time_;
    bool safety_ok_ = true;
    std::unordered_map<std::string, double> current_positions_;
    std::unordered_map<std::string, double> stand_start_positions_;

    struct WiggleJoint { double pos; double neg; double freq; };
    struct WiggleCfg { double duration; std::unordered_map<std::string, WiggleJoint> joints; };
    WiggleCfg wiggle_cfg_;
    double wiggle_start_ = 0.0;
    int wiggle_joint_idx_ = 0;

    int active_joint_idx_ = -1;
    int target_joint_idx_ = -1;
    bool wiggle_interpolating_ = false;
    double interp_start_time_ = 0.0;
    double interp_start_pos_ = 0.0;
    double wiggle_sine_start_time_ = 0.0;

    std::vector<std::vector<double>> traj_frames_;
    int traj_index_ = 0;
    bool traj_sim_only_ = false;
    double traj_gain_ramp_secs_ = 3.0;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<biped_msgs::msg::RobotState>::SharedPtr pub_robot_;
    rclcpp::Publisher<biped_msgs::msg::MITCommandArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_viz_js_;
    
    rclcpp::TimerBase::SharedPtr timer_control_;
    rclcpp::TimerBase::SharedPtr timer_status_;

    void load_wiggle_config() {
        std::string path = get_parameter("wiggle_config").as_string();
        wiggle_cfg_.joints.clear();
        wiggle_cfg_.duration = 3.0;
        double global_freq = 1.0;

        if (!path.empty()) {
            try {
                YAML::Node raw = YAML::LoadFile(path);
                if (raw["wiggle"]) {
                    auto w = raw["wiggle"];
                    if (w["duration_per_joint"]) wiggle_cfg_.duration = w["duration_per_joint"].as<double>();
                    
                    if (w["joints"]) {
                        for (auto it = w["joints"].begin(); it != w["joints"].end(); ++it) {
                            std::string name = it->first.as<std::string>();
                            auto params = it->second;
                            double pos = params["pos"] ? params["pos"].as<double>() * M_PI / 180.0 : 0.087;
                            double neg = params["neg"] ? params["neg"].as<double>() * M_PI / 180.0 : -0.087;
                            double freq = params["freq"] ? params["freq"].as<double>() : global_freq;

                            auto lit = JOINT_LIMITS.find(name);
                            if (lit != JOINT_LIMITS.end()) {
                                double lo = lit->second.first;
                                double hi = lit->second.second;
                                double def = DEFAULT_POSITIONS.at(name);
                                if (def + pos > hi) {
                                    pos = hi - def;
                                    RCLCPP_WARN(get_logger(), "%s: pos clamped to %.3f", name.c_str(), pos);
                                }
                                if (def + neg < lo) {
                                    neg = lo - def;
                                    RCLCPP_WARN(get_logger(), "%s: neg clamped to %.3f", name.c_str(), neg);
                                }
                            }
                            wiggle_cfg_.joints[name] = {pos, neg, freq};
                        }
                    }
                }
                RCLCPP_INFO(get_logger(), "Wiggle config loaded");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Failed to load wiggle config: %s", e.what());
            }
        }
        for (const auto& name : JOINT_ORDER) {
            if (wiggle_cfg_.joints.find(name) == wiggle_cfg_.joints.end()) {
                wiggle_cfg_.joints[name] = {0.087, -0.087, global_freq};
            }
        }
    }

    std::vector<std::string> split(const std::string& s, char delim) {
        std::vector<std::string> result;
        size_t start = 0;
        size_t end = s.find(delim);
        while (end != std::string::npos) {
            result.push_back(s.substr(start, end - start));
            start = end + 1;
            end = s.find(delim, start);
        }
        result.push_back(s.substr(start));
        return result;
    }

    void load_trajectory() {
        std::string path = get_parameter("trajectory_file").as_string();
        traj_frames_.clear();
        traj_index_ = 0;

        if (path.empty()) {
            RCLCPP_ERROR(get_logger(), "Trajectory file empty");
            transition("STAND");
            return;
        }
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Trajectory file not found: %s", path.c_str());
            transition("STAND");
            return;
        }

        std::vector<double> t_csv;
        std::vector<std::vector<double>> joints_csv(12);

        std::string line;
        int row_idx = 0;
        while (std::getline(file, line)) {
            auto cols = split(line, ',');
            if (row_idx == 0) {
                for (auto& c : cols) if (!c.empty()) t_csv.push_back(std::stod(c));
            } else if (row_idx <= 12) {
                for (auto& c : cols) if (!c.empty()) joints_csv[row_idx - 1].push_back(std::stod(c));
            }
            row_idx++;
        }

        if (t_csv.empty() || joints_csv[0].size() != t_csv.size()) {
            RCLCPP_ERROR(get_logger(), "Corrupt trajectory CSV");
            transition("STAND");
            return;
        }

        double dt_ctrl = 0.02;
        int num_frames = static_cast<int>(t_csv.back() / dt_ctrl);
        std::vector<std::vector<double>> joints_resampled(12, std::vector<double>(num_frames));

        for (int i = 0; i < 12; i++) {
            for (int t = 0; t < num_frames; t++) {
                double target_t = t * dt_ctrl;
                auto it = std::lower_bound(t_csv.begin(), t_csv.end(), target_t);
                if (it == t_csv.end()) {
                    joints_resampled[i][t] = joints_csv[i].back();
                } else if (it == t_csv.begin()) {
                    joints_resampled[i][t] = joints_csv[i].front();
                } else {
                    auto prev = std::prev(it);
                    double t0 = *prev;
                    double t1 = *it;
                    double v0 = joints_csv[i][std::distance(t_csv.begin(), prev)];
                    double v1 = joints_csv[i][std::distance(t_csv.begin(), it)];
                    joints_resampled[i][t] = v0 + (v1 - v0) * ((target_t - t0) / (t1 - t0));
                }
            }
        }

        std::vector<int> csv_to_deploy;
        for (const auto& name : JOINT_ORDER) {
            auto it = std::find(CSV_JOINT_ORDER.begin(), CSV_JOINT_ORDER.end(), name);
            csv_to_deploy.push_back(std::distance(CSV_JOINT_ORDER.begin(), it));
        }

        int ramp_frames = static_cast<int>(3.0 / dt_ctrl);
        for (int i = 0; i < ramp_frames; i++) {
            std::vector<double> frame(12);
            double alpha = static_cast<double>(i + 1) / ramp_frames;
            alpha = 0.5 * (1.0 - std::cos(M_PI * alpha));
            for (int j = 0; j < 12; j++) {
                std::string name = JOINT_ORDER[j];
                double start = current_positions_.at(name);
                double end = joints_resampled[csv_to_deploy[j]][0];
                frame[j] = start + alpha * (end - start);
            }
            traj_frames_.push_back(frame);
        }

        for (int t = 0; t < num_frames; t++) {
            std::vector<double> frame(12);
            for (int j = 0; j < 12; j++) {
                frame[j] = joints_resampled[csv_to_deploy[j]][t];
            }
            traj_frames_.push_back(frame);
        }

        RCLCPP_INFO(get_logger(), "Trajectory loaded: %zu total frames", traj_frames_.size());
    }

    void safety_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        safety_ok_ = msg->data;
        if (!safety_ok_ && state_ != "IDLE" && state_ != "ESTOP") {
            transition("ESTOP");
        }
    }

    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size()) {
                current_positions_[msg->name[i]] = msg->position[i];
            }
        }
    }

    void command_cb(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        // uppercase
        for (auto& c : cmd) c = std::toupper(c);

        if (cmd == "START" && state_ == "IDLE") transition("STAND");
        else if (cmd == "WALK" && state_ == "STAND") transition("WALK");
        else if (cmd == "SIM_WALK" && state_ == "STAND") transition("SIM_WALK");
        else if (cmd == "PLAY_TRAJ" && state_ == "STAND") transition("PLAY_TRAJ");
        else if (cmd == "PLAY_TRAJ_SIM" && state_ == "STAND") transition("PLAY_TRAJ_SIM");
        else if (cmd == "STOP" && state_ != "IDLE" && state_ != "ESTOP") transition("STAND");
        else if (cmd.find("WIGGLE_SEQ") == 0) {
            size_t colon_pos = cmd.find(":");
            if (colon_pos != std::string::npos) {
                int target_idx = std::stoi(cmd.substr(colon_pos + 1));
                if (target_idx >= 0 && target_idx < static_cast<int>(JOINT_ORDER.size())) {
                    if (state_ != "WIGGLE_SEQ") {
                        transition("WIGGLE_SEQ");
                    }
                    if (target_idx != active_joint_idx_) {
                        target_joint_idx_ = target_idx;
                        interp_start_time_ = now().seconds();
                        if (active_joint_idx_ >= 0) {
                            interp_start_pos_ = current_positions_[JOINT_ORDER[active_joint_idx_]] - DEFAULT_POSITIONS.at(JOINT_ORDER[active_joint_idx_]);
                            wiggle_interpolating_ = true;
                        } else {
                            // If it's the very first joint, jump straight in
                            active_joint_idx_ = target_joint_idx_;
                            wiggle_sine_start_time_ = now().seconds();
                            wiggle_interpolating_ = false;
                        }
                    }
                }
            } else {
                if (state_ != "WIGGLE_SEQ") transition("WIGGLE_SEQ");
            }
        }
        else if (cmd == "WIGGLE_ALL" && state_ == "STAND") transition("WIGGLE_ALL");
        else if (cmd == "ESTOP") transition("ESTOP");
        else if (cmd == "RESET" && state_ == "ESTOP") transition("IDLE");
    }

    void transition(const std::string& new_state) {
        std::string old = state_;
        state_ = new_state;
        state_start_time_ = now().seconds();

        if (new_state == "STAND") {
            stand_start_positions_ = current_positions_;
        } else if (new_state == "PLAY_TRAJ" || new_state == "PLAY_TRAJ_SIM") {
            traj_sim_only_ = (new_state == "PLAY_TRAJ_SIM");
            load_trajectory();
        } else if (new_state == "WIGGLE_SEQ") {
            stand_start_positions_ = current_positions_;
            load_wiggle_config();
            active_joint_idx_ = -1;
            target_joint_idx_ = -1;
            wiggle_interpolating_ = false;
        } else if (new_state == "WIGGLE_ALL") {
            stand_start_positions_ = current_positions_;
            load_wiggle_config();
            wiggle_start_ = now().seconds();
        } else if (new_state == "ESTOP") {
            send_zero_torque();
        }
        RCLCPP_INFO(get_logger(), "State: %s -> %s", old.c_str(), new_state.c_str());
    }

    void loop_cb() {
        if (state_ == "STAND") handle_stand();
        else if (state_ == "WALK") {} // Policy node handles WALK
        else if (state_ == "SIM_WALK") handle_stand_hold();
        else if (state_ == "WIGGLE_SEQ") handle_wiggle_sequential();
        else if (state_ == "WIGGLE_ALL") handle_wiggle_all();
        else if (state_ == "PLAY_TRAJ") handle_play_traj();
        else if (state_ == "PLAY_TRAJ_SIM") { handle_stand_hold(); handle_play_traj_sim(); }
        else if (state_ == "ESTOP") send_zero_torque();
    }

    void handle_stand() {
        double elapsed = now().seconds() - state_start_time_;
        double pos_alpha = std::min(elapsed / ramp_time_, 1.0);
        double gain_elapsed = std::max(0.0, elapsed - ramp_time_);
        double gain_alpha = std::min(gain_elapsed / gain_ramp_time_, 1.0);

        double soft_kp_scale = 0.1 + 0.9 * gain_alpha;
        double soft_kd_scale = 0.2 + 0.8 * gain_alpha;
        double gs = get_parameter("gain_scale").as_double();

        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            double start_pos = stand_start_positions_.count(name) ? stand_start_positions_[name] : DEFAULT_POSITIONS.at(name);
            double target_pos = DEFAULT_POSITIONS.at(name);
            double current_target = start_pos + (target_pos - start_pos) * pos_alpha;
            
            auto gains = DEFAULT_GAINS.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = current_target;
            cmd.velocity = 0.0;
            cmd.kp = gains.first * soft_kp_scale * gs;
            cmd.kd = gains.second * soft_kd_scale * gs;
            cmd.torque_ff = 0.0;
            msg.commands.push_back(cmd);
        }
        pub_cmd_->publish(msg);
        pub_vel_->publish(geometry_msgs::msg::Twist());
    }

    void handle_stand_hold() {
        double gs = get_parameter("gain_scale").as_double();
        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            auto gains = DEFAULT_GAINS.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = DEFAULT_POSITIONS.at(name);
            cmd.velocity = 0.0;
            cmd.kp = gains.first * gs;
            cmd.kd = gains.second * gs;
            cmd.torque_ff = 0.0;
            msg.commands.push_back(cmd);
        }
        pub_cmd_->publish(msg);
    }

    void handle_wiggle_sequential() {
        double elapsed = now().seconds() - state_start_time_;
        
        // Initial entry ramp (same as STAND)
        if (elapsed <= ramp_time_ + stable_time_) {
            handle_stand();
            return;
        }

        double gs = get_parameter("gain_scale").as_double();
        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();

        double current_time = now().seconds();

        // Handle Interpolation
        if (wiggle_interpolating_) {
            double alpha = (current_time - interp_start_time_) / 3.0; // 3 seconds grace
            if (alpha >= 1.0) {
                alpha = 1.0;
                wiggle_interpolating_ = false;
                active_joint_idx_ = target_joint_idx_;
                wiggle_sine_start_time_ = current_time;
            }
            
            for (int i = 0; i < static_cast<int>(JOINT_ORDER.size()); ++i) {
                std::string name = JOINT_ORDER[i];
                double target = DEFAULT_POSITIONS.at(name);

                if (i == active_joint_idx_) {
                    // Pull old joint back to 0.0 relative to default
                    target += interp_start_pos_ * (1.0 - alpha);
                }

                auto lit = JOINT_LIMITS.find(name);
                if (lit != JOINT_LIMITS.end()) {
                    target = std::max(lit->second.first, std::min(lit->second.second, target));
                }

                auto gains = DEFAULT_GAINS.at(name);
                biped_msgs::msg::MITCommand cmd;
                cmd.joint_name = name;
                cmd.position = target;
                cmd.velocity = 0.0;
                cmd.kp = gains.first * gs;
                cmd.kd = gains.second * gs;
                cmd.torque_ff = 0.0;
                msg.commands.push_back(cmd);
            }
        } 
        else {
            // Active Wiggle
            for (int i = 0; i < static_cast<int>(JOINT_ORDER.size()); ++i) {
                std::string name = JOINT_ORDER[i];
                double target = DEFAULT_POSITIONS.at(name);

                if (i == active_joint_idx_) {
                    auto& jcfg = wiggle_cfg_.joints[name];
                    double phase_t = current_time - wiggle_sine_start_time_;
                    double sin_val = std::sin(2 * M_PI * jcfg.freq * phase_t);
                    double offset = (sin_val >= 0) ? (jcfg.pos * sin_val) : (-jcfg.neg * sin_val);
                    target += offset;
                }

                auto lit = JOINT_LIMITS.find(name);
                if (lit != JOINT_LIMITS.end()) {
                    target = std::max(lit->second.first, std::min(lit->second.second, target));
                }

                auto gains = DEFAULT_GAINS.at(name);
                biped_msgs::msg::MITCommand cmd;
                cmd.joint_name = name;
                cmd.position = target;
                cmd.velocity = 0.0;
                cmd.kp = gains.first * gs;
                cmd.kd = gains.second * gs;
                cmd.torque_ff = 0.0;
                msg.commands.push_back(cmd);
            }
        }

        pub_cmd_->publish(msg);
    }

    void handle_wiggle_all() {
        double elapsed = now().seconds() - state_start_time_;
        
        // Initial entry ramp (same as STAND)
        if (elapsed <= ramp_time_ + stable_time_) {
            handle_stand();
            return;
        }
        
        double gs = get_parameter("gain_scale").as_double();
        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            double target = DEFAULT_POSITIONS.at(name);
            auto& jcfg = wiggle_cfg_.joints[name];
            double sin_val = std::sin(2 * M_PI * jcfg.freq * (now().seconds() - wiggle_start_));
            double offset = (sin_val >= 0) ? (jcfg.pos * sin_val) : (-jcfg.neg * sin_val);
            target += offset;

            auto lit = JOINT_LIMITS.find(name);
            if (lit != JOINT_LIMITS.end()) {
                target = std::max(lit->second.first, std::min(lit->second.second, target));
            }

            auto gains = DEFAULT_GAINS.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = target;
            cmd.velocity = 0.0;
            cmd.kp = gains.first * gs;
            cmd.kd = gains.second * gs;
            cmd.torque_ff = 0.0;
            msg.commands.push_back(cmd);
        }
        pub_cmd_->publish(msg);
    }

    void handle_play_traj() {
        if (traj_frames_.empty()) return;
        if (traj_index_ >= static_cast<int>(traj_frames_.size())) {
            RCLCPP_INFO(get_logger(), "Trajectory playback complete");
            transition("STAND");
            return;
        }

        auto frame = traj_frames_[traj_index_];
        double elapsed = now().seconds() - state_start_time_;
        double gain_ramp = std::min(1.0, 0.1 + 0.9 * (elapsed / traj_gain_ramp_secs_));
        double gs = get_parameter("gain_scale").as_double();

        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();

        for (int j = 0; j < 12; j++) {
            std::string name = JOINT_ORDER[j];
            auto gains = DEFAULT_GAINS.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = frame[j];
            cmd.velocity = 0.0;
            cmd.kp = gains.first * gs * gain_ramp;
            cmd.kd = gains.second * gs * gain_ramp;
            cmd.torque_ff = 0.0;
            msg.commands.push_back(cmd);
        }
        pub_cmd_->publish(msg);

        if (traj_index_ % 50 == 0) {
            RCLCPP_INFO(get_logger(), "Traj %d/%zu", traj_index_, traj_frames_.size());
        }
        traj_index_++;
    }

    void handle_play_traj_sim() {
        if (traj_frames_.empty()) return;
        if (traj_index_ >= static_cast<int>(traj_frames_.size())) {
            RCLCPP_INFO(get_logger(), "Trajectory SIM playback complete");
            transition("STAND");
            return;
        }

        auto frame = traj_frames_[traj_index_];
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        
        for (int j = 0; j < 12; j++) {
            js.name.push_back(JOINT_ORDER[j]);
            js.position.push_back(frame[j]);
            js.velocity.push_back(0.0);
            js.effort.push_back(0.0);
        }
        pub_viz_js_->publish(js);

        if (traj_index_ % 50 == 0) {
            RCLCPP_INFO(get_logger(), "Traj SIM %d/%zu", traj_index_, traj_frames_.size());
        }
        traj_index_++;
    }

    void send_zero_torque() {
        biped_msgs::msg::MITCommandArray msg;
        msg.header.stamp = now();
        for (const auto& name : JOINT_ORDER) {
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = 0.0;
            cmd.velocity = 0.0;
            cmd.kp = 0.0;
            cmd.kd = 0.0;
            cmd.torque_ff = 0.0;
            msg.commands.push_back(cmd);
        }
        pub_cmd_->publish(msg);
    }

    void publish_state_cb() {
        std_msgs::msg::String sm;
        sm.data = state_;
        pub_state_->publish(sm);

        biped_msgs::msg::RobotState rm;
        rm.header.stamp = now();
        rm.fsm_state = state_;
        rm.safety_ok = safety_ok_;
        rm.all_motors_enabled = (state_ != "IDLE" && state_ != "ESTOP");
        pub_robot_->publish(rm);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNodeCpp>());
    rclcpp::shutdown();
    return 0;
}
