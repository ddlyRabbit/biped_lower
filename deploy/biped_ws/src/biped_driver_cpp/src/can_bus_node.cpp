/**
 * @file can_bus_node.cpp
 * @brief ROS2 CAN bus node — C++ drop-in replacement for Python can_bus_node.
 *
 * Two worker threads (one per CAN bus) running tight send→recv loops.
 * ROS2 timer publishes /joint_states and /motor_states at publish_rate.
 * Subscribes to /joint_commands (biped_msgs/MITCommandArray).
 * Ankle joints handled transparently via parallel linkage transform.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <biped_msgs/msg/mit_command.hpp>
#include <biped_msgs/msg/mit_command_array.hpp>
#include <biped_msgs/msg/motor_state.hpp>
#include <biped_msgs/msg/motor_state_array.hpp>

#include <yaml-cpp/yaml.h>

#include "biped_driver_cpp/motor_manager.hpp"
#include "biped_driver_cpp/ankle_linkage.hpp"

namespace biped_driver_cpp {

// URDF joint-space limits for ankle joints
static constexpr double ANKLE_PITCH_LO = -0.87267;
static constexpr double ANKLE_PITCH_HI =  0.52360;
static constexpr double ANKLE_ROLL_LO  = -0.26180;
static constexpr double ANKLE_ROLL_HI  =  0.26180;

// ── Shared buffer types ──────────────────────────────────────────

struct MotorCommand {
    double position = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double velocity = 0.0;
    double torque_ff = 0.0;
    bool valid = false;
};

struct FeedbackEntry {
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    double temperature = 0.0;
    int fault_code = 0;
    int mode_status = 0;
    bool valid = false;
};

class SharedBuffer {
public:
    explicit SharedBuffer(const std::vector<std::string>& names) {
        for (auto& n : names) {
            commands_[n] = MotorCommand{};
            feedback_[n] = FeedbackEntry{};
        }
    }

    void write_command(const std::string& name, const MotorCommand& cmd) {
        std::lock_guard<std::mutex> lk(mtx_);
        commands_[name] = cmd;
    }

    void notify_new_commands() {
        {
            std::lock_guard<std::mutex> lk(cmd_cv_mtx_);
            new_command_ready_ = true;
        }
        cmd_cv_.notify_all();
    }

    /// Block until a command arrives or timeout_ms elapses.
    /// Returns true if woken by command, false on timeout.
    bool wait_for_command(int timeout_ms = 0) {
        std::unique_lock<std::mutex> lk(cmd_cv_mtx_);
        bool woken_by_cmd = cmd_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), 
                                             [this] { return new_command_ready_; });
        new_command_ready_ = false;
        return woken_by_cmd;
    }

    std::unordered_map<std::string, MotorCommand> read_commands() {
        std::lock_guard<std::mutex> lk(mtx_);
        return commands_;
    }

    void write_feedback(const std::string& name, const FeedbackEntry& fb) {
        std::lock_guard<std::mutex> lk(mtx_);
        feedback_[name] = fb;
    }

    std::unordered_map<std::string, FeedbackEntry> read_all_feedback() {
        std::lock_guard<std::mutex> lk(mtx_);
        return feedback_;
    }

    void update_stats(double dt) {
        std::lock_guard<std::mutex> lk(mtx_);
        loop_dt_ = dt;
        loop_count_++;
    }

    double loop_dt() { std::lock_guard<std::mutex> lk(mtx_); return loop_dt_; }
    uint64_t loop_count() { std::lock_guard<std::mutex> lk(mtx_); return loop_count_; }

private:
    std::mutex mtx_;
    std::condition_variable cmd_cv_;
    std::mutex cmd_cv_mtx_;
    std::unordered_map<std::string, MotorCommand> commands_;
    std::unordered_map<std::string, FeedbackEntry> feedback_;
    double loop_dt_ = 0.0;
    uint64_t loop_count_ = 0;
    bool new_command_ready_ = false;
};

// ── CAN Worker Thread ────────────────────────────────────────────

class CanBusWorker {
public:
    CanBusWorker(const std::string& bus_name,
                 BipedMotorManager& mgr,
                 const std::vector<std::string>& motor_names,
                 std::shared_ptr<SharedBuffer> buffer,
                 rclcpp::Logger logger)
        : bus_name_(bus_name), mgr_(mgr), motor_names_(motor_names),
          buffer_(buffer), logger_(logger),
          clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)) {
        build_motor_groups();
    }

    void start() {
        running_ = true;
        thread_ = std::thread(&CanBusWorker::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    double target_rate_hz_;

    void build_motor_groups() {
        std::unordered_set<std::string> seen;
        std::unordered_set<std::string> motor_set(motor_names_.begin(), motor_names_.end());

        for (auto& name : motor_names_) {
            if (seen.count(name)) continue;
            auto pair = mgr_.get_ankle_pair(name);
            if (pair && motor_set.count(pair->first) && motor_set.count(pair->second)) {
                ankle_pairs_.push_back(*pair);
                seen.insert(pair->first);
                seen.insert(pair->second);
            } else if (!seen.count(name)) {
                normal_motors_.push_back(name);
                seen.insert(name);
            }
        }
    }

    void run() {
        RCLCPP_INFO(logger_, "[%s] Worker started: %zu normal + %zu ankle pairs (event-driven, 25ms timeout)",
                     bus_name_.c_str(), normal_motors_.size(), ankle_pairs_.size());

        auto last_t0 = std::chrono::steady_clock::now();

        while (running_) {
            // Wait for command or 5ms timeout — avoids busy spinning
            buffer_->wait_for_command(5);

            auto t0 = std::chrono::steady_clock::now();
            auto commands = buffer_->read_commands();

            // ── Phase 1: Send all commands ───────────────────────
            for (auto& name : normal_motors_) {
                try {
                    auto it = commands.find(name);
                    if (it != commands.end() && it->second.valid) {
                        auto& cmd = it->second;
                        mgr_.send_mit_command(name, cmd.position, cmd.kp, cmd.kd,
                                              cmd.velocity, cmd.torque_ff,
                                              last_pos(name));
                    } else {
                        mgr_.send_mit_command(name, 0.0, 0.0, 0.0);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_,
                                         1000, "[%s] %s send error: %s",
                                         bus_name_.c_str(), name.c_str(), e.what());
                }
            }

            for (auto& [top_name, bottom_name] : ankle_pairs_) {
                auto& amj = ankle_motor_to_joint();
                std::string pitch_joint = amj.at(top_name);
                std::string roll_joint = amj.at(bottom_name);
                int pitch_sign = (top_name[0] == 'L') ? -1 : 1;

                try {
                    auto pit = commands.find(pitch_joint);
                    auto rit = commands.find(roll_joint);
                    if (pit != commands.end() && pit->second.valid &&
                        rit != commands.end() && rit->second.valid) {
                        auto& pcmd = pit->second;
                        auto& rcmd = rit->second;

                        double pitch_pos = std::max(ANKLE_PITCH_LO,
                                           std::min(ANKLE_PITCH_HI, pcmd.position));
                        double roll_pos = std::max(ANKLE_ROLL_LO,
                                          std::min(ANKLE_ROLL_HI, rcmd.position));

                        auto [motor_upper, motor_lower] =
                            ankle_command_to_motors(pitch_pos, roll_pos, pitch_sign);

                        double kp = (pcmd.kp + rcmd.kp) / 2.0;
                        double kd = (pcmd.kd + rcmd.kd) / 2.0;
                        double tff = (pcmd.torque_ff + rcmd.torque_ff) / 2.0;

                        mgr_.send_ankle_mit_command(top_name, motor_upper, kp, kd,
                                                    0.0, tff, last_pos(top_name));
                        mgr_.send_ankle_mit_command(bottom_name, motor_lower, kp, kd,
                                                    0.0, tff, last_pos(bottom_name));
                    } else {
                        mgr_.send_ankle_mit_command(top_name, 0.0, 0.0, 0.0);
                        mgr_.send_ankle_mit_command(bottom_name, 0.0, 0.0, 0.0);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_,
                                         1000, "[%s] ankle %s/%s send error: %s",
                                         bus_name_.c_str(), top_name.c_str(),
                                         bottom_name.c_str(), e.what());
                }
            }

            // ── Phase 2: Read all feedback ───────────────────────
            for (auto& name : normal_motors_) {
                try {
                    auto fb = mgr_.read_feedback(name);
                    if (fb) {
                        last_positions_[name] = fb->position;
                        buffer_->write_feedback(name, FeedbackEntry{
                            fb->position, fb->velocity, fb->torque,
                            fb->temperature, fb->fault_code, fb->mode_status, true
                        });
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_,
                                         1000, "[%s] %s read error: %s",
                                         bus_name_.c_str(), name.c_str(), e.what());
                }
            }

            for (auto& [top_name, bottom_name] : ankle_pairs_) {
                for (auto& motor_name : {top_name, bottom_name}) {
                    try {
                        auto fb = mgr_.read_feedback(motor_name);
                        if (fb) {
                            last_positions_[motor_name] = fb->position;
                            buffer_->write_feedback(motor_name, FeedbackEntry{
                                fb->position, fb->velocity, fb->torque,
                                fb->temperature, fb->fault_code, fb->mode_status, true
                            });
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN_THROTTLE(logger_, *clock_,
                                             1000, "[%s] %s read error: %s",
                                             bus_name_.c_str(), motor_name.c_str(), e.what());
                    }
                }
            }

            auto dt = std::chrono::duration<double>(t0 - last_t0).count();
            last_t0 = t0;
            buffer_->update_stats(dt);
        }
    }

    std::optional<double> last_pos(const std::string& name) {
        auto it = last_positions_.find(name);
        if (it != last_positions_.end()) return it->second;
        return std::nullopt;
    }

    std::string bus_name_;
    BipedMotorManager& mgr_;
    std::vector<std::string> motor_names_;
    std::shared_ptr<SharedBuffer> buffer_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::vector<std::string> normal_motors_;
    std::vector<std::pair<std::string, std::string>> ankle_pairs_;
    std::unordered_map<std::string, double> last_positions_;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

// ── ROS2 Node ────────────────────────────────────────────────────

class CanBusNodeCpp : public rclcpp::Node {
public:
    CanBusNodeCpp() : Node("can_bus_node") {
        // ── Parameters ───────────────────────────────────────────
        declare_parameter("robot_config", "");
        declare_parameter("calibration_file", "");
        declare_parameter("publish_rate", 200.0);

        std::string robot_config_path = get_parameter("robot_config").as_string();
        std::string cal_file = get_parameter("calibration_file").as_string();
        pub_rate_ = get_parameter("publish_rate").as_double();

        // ── Load YAML ────────────────────────────────────────────
        YAML::Node robot_yaml, cal_yaml;
        if (!robot_config_path.empty()) {
            try {
                robot_yaml = YAML::LoadFile(robot_config_path);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to load robot config: %s", e.what());
                return;
            }
        } else {
            RCLCPP_ERROR(get_logger(), "robot_config parameter required");
            return;
        }

        if (!cal_file.empty()) {
            try {
                cal_yaml = YAML::LoadFile(cal_file);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "No calibration loaded: %s", e.what());
            }
        }

        // ── Build motor manager ──────────────────────────────────
        mgr_ = std::make_unique<BipedMotorManager>(
            build_manager_from_yaml(robot_yaml, cal_yaml));

        // Build reverse map: joint-space → motor name
        for (auto& [motor, joint] : ankle_motor_to_joint()) {
            joint_to_motor_[joint] = motor;
        }

        // ── Group motors by bus ──────────────────────────────────
        std::unordered_map<std::string, std::vector<std::string>> bus_motors;
        for (auto& [name, jcfg] : mgr_->joints()) {
            bus_motors[jcfg.can_bus].push_back(name);
        }

        // ── Create shared buffers + workers ──────────────────────
        for (auto& [bus_name, motors] : bus_motors) {
            // Buffer uses joint-space names for ankle motors
            std::vector<std::string> buffer_names;
            for (auto& name : motors) {
                auto amj_it = ankle_motor_to_joint().find(name);
                if (amj_it != ankle_motor_to_joint().end()) {
                    buffer_names.push_back(amj_it->second);
                } else {
                    buffer_names.push_back(name);
                }
            }
            // Also add motor names for ankle feedback
            for (auto& name : motors) {
                if (is_ankle_motor(name)) {
                    buffer_names.push_back(name);
                }
            }

            auto buf = std::make_shared<SharedBuffer>(buffer_names);
            buffers_[bus_name] = buf;
            workers_.push_back(std::make_unique<CanBusWorker>(
                bus_name, *mgr_, motors, buf, get_logger()));
        }

        // ── QoS + Pub/Sub ────────────────────────────────────────
        auto qos = rclcpp::QoS(1).best_effort();
        pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
        pub_motors_ = create_publisher<biped_msgs::msg::MotorStateArray>("/motor_states", qos);
        sub_commands_ = create_subscription<biped_msgs::msg::MITCommandArray>(
            "/joint_commands", 10,
            std::bind(&CanBusNodeCpp::cmd_callback, this, std::placeholders::_1));

        // ── Log motor map ────────────────────────────────────────
        for (auto& [bus_name, motors] : bus_motors) {
            std::string names;
            for (auto& n : motors) {
                if (!names.empty()) names += ", ";
                names += n;
            }
            RCLCPP_INFO(get_logger(), "  %s: %s", bus_name.c_str(), names.c_str());
        }
        size_t total = mgr_->joints().size();
        RCLCPP_INFO(get_logger(), "Total: %zu motors on %zu bus(es)",
                     total, mgr_->buses().size());

        // ── Connect, enable, start workers ───────────────────────
        mgr_->connect_all();
        mgr_->flush_all();
        mgr_->enable_all();
        RCLCPP_INFO(get_logger(), "All motors enabled in MIT mode.");

        for (auto& w : workers_) w->start();
        RCLCPP_INFO(get_logger(), "Started %zu CAN worker threads.", workers_.size());

        // ── Publish timer ────────────────────────────────────────
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / pub_rate_),
            std::bind(&CanBusNodeCpp::publish_loop, this));

        // ── Stats timer (every 5s) ──────────────────────────────
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&CanBusNodeCpp::log_stats, this));

        RCLCPP_INFO(get_logger(), "C++ CAN bus node ready — publish @ %.0fHz", pub_rate_);
    }

    ~CanBusNodeCpp() override {
        RCLCPP_INFO(get_logger(), "Shutting down C++ CAN node");
        for (auto& w : workers_) w->stop();
        try { mgr_->disable_all(); } catch (...) {}
        mgr_->disconnect_all();
    }

private:
    // ── Command callback ─────────────────────────────────────────
    void cmd_callback(const biped_msgs::msg::MITCommandArray::SharedPtr msg) {
        for (auto& cmd : msg->commands) {
            std::string name = cmd.joint_name;
            MotorCommand mc{cmd.position, cmd.kp, cmd.kd,
                            cmd.velocity, cmd.torque_ff, true};

            // Accept both motor names and joint-space names
            bool found = false;
            for (auto& [_, buf] : buffers_) {
                // Try direct name first
                auto cmds = buf->read_commands();
                if (cmds.count(name)) {
                    buf->write_command(name, mc);
                    found = true;
                    break;
                }
            }
            if (!found) {
                // Try as motor name mapped to joint
                auto amj_it = ankle_motor_to_joint().find(name);
                if (amj_it != ankle_motor_to_joint().end()) {
                    for (auto& [_, buf] : buffers_) {
                        auto cmds = buf->read_commands();
                        if (cmds.count(amj_it->second)) {
                            buf->write_command(amj_it->second, mc);
                            break;
                        }
                    }
                }
            }
        }

        // Notify worker loops that a new command array has been written
        for (auto& [_, buf] : buffers_) {
            buf->notify_new_commands();
        }
    }

    // ── Publish loop ─────────────────────────────────────────────
    void publish_loop() {
        auto now = this->now();
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = now;
        auto motor_msg = biped_msgs::msg::MotorStateArray();
        motor_msg.header.stamp = now;

        for (auto& [bus_name, buf] : buffers_) {
            auto feedback = buf->read_all_feedback();

            // Get motor names on this bus
            std::vector<std::string> bus_motor_names;
            for (auto& [name, jcfg] : mgr_->joints()) {
                if (jcfg.can_bus == bus_name) {
                    bus_motor_names.push_back(name);
                }
            }

            std::unordered_set<std::string> processed;
            for (auto& name : bus_motor_names) {
                if (processed.count(name)) continue;

                auto pair = mgr_->get_ankle_pair(name);
                if (pair) {
                    auto& [top_name, bottom_name] = *pair;
                    // Check both are on this bus
                    int found = 0;
                    for (auto& n : bus_motor_names) {
                        if (n == top_name || n == bottom_name) found++;
                    }
                    bool both_here = (found == 2);

                    auto fb_top = feedback.count(top_name) ? feedback[top_name] : FeedbackEntry{};
                    auto fb_bot = feedback.count(bottom_name) ? feedback[bottom_name] : FeedbackEntry{};

                    if (fb_top.valid && fb_bot.valid) {
                        int pitch_sign = (top_name[0] == 'L') ? -1 : 1;
                        auto [foot_pitch, foot_roll] = ankle_motors_to_feedback(
                            fb_top.position, fb_bot.position, pitch_sign);
                        auto [foot_pitch_vel, foot_roll_vel] = ankle_motors_to_feedback(
                            fb_top.velocity, fb_bot.velocity, pitch_sign);

                        auto& amj = ankle_motor_to_joint();
                        std::string pitch_joint = amj.at(top_name);
                        std::string roll_joint = amj.at(bottom_name);

                        // Pitch joint (joint-space)
                        append_joint(joint_msg, pitch_joint, foot_pitch,
                                     foot_pitch_vel, fb_top.torque);
                        append_motor(motor_msg, top_name, fb_top);

                        // Roll joint (joint-space)
                        append_joint(joint_msg, roll_joint, foot_roll,
                                     foot_roll_vel, fb_bot.torque);
                        append_motor(motor_msg, bottom_name, fb_bot);
                    }

                    processed.insert(top_name);
                    processed.insert(bottom_name);
                } else {
                    auto fb_it = feedback.find(name);
                    if (fb_it != feedback.end() && fb_it->second.valid) {
                        auto& fb = fb_it->second;
                        append_joint(joint_msg, name, fb.position, fb.velocity, fb.torque);
                        append_motor(motor_msg, name, fb);
                    }
                    processed.insert(name);
                }
            }
        }

        pub_joints_->publish(joint_msg);
        pub_motors_->publish(motor_msg);
    }

    void append_joint(sensor_msgs::msg::JointState& msg,
                      const std::string& name, double pos, double vel, double effort) {
        msg.name.push_back(name);
        msg.position.push_back(pos);
        msg.velocity.push_back(vel);
        msg.effort.push_back(effort);
    }

    void append_motor(biped_msgs::msg::MotorStateArray& msg,
                      const std::string& name, const FeedbackEntry& fb) {
        biped_msgs::msg::MotorState ms;
        ms.joint_name = name;
        auto jit = mgr_->joints().find(name);
        if (jit != mgr_->joints().end()) {
            ms.can_id = static_cast<uint8_t>(jit->second.can_id);
        }
        ms.position = fb.position;
        ms.velocity = fb.velocity;
        ms.torque = fb.torque;
        ms.temperature = fb.temperature;
        ms.fault_code = fb.fault_code;
        ms.mode_status = fb.mode_status;
        msg.motors.push_back(ms);
    }

    // ── Stats logging ────────────────────────────────────────────
    void log_stats() {
        for (auto& [bus_name, buf] : buffers_) {
            double dt = buf->loop_dt();
            uint64_t count = buf->loop_count();
            if (dt > 0) {
                double hz = 1.0 / dt;
                RCLCPP_INFO(get_logger(), "[%s] CAN loop: %.0fHz (dt=%.1fms, total=%lu)",
                            bus_name.c_str(), hz, dt * 1000.0, count);
            }
        }
    }

    // Members
    std::unique_ptr<BipedMotorManager> mgr_;
    double pub_rate_ = 50.0;

    std::unordered_map<std::string, std::string> joint_to_motor_;
    std::unordered_map<std::string, std::shared_ptr<SharedBuffer>> buffers_;
    std::vector<std::unique_ptr<CanBusWorker>> workers_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
    rclcpp::Publisher<biped_msgs::msg::MotorStateArray>::SharedPtr pub_motors_;
    rclcpp::Subscription<biped_msgs::msg::MITCommandArray>::SharedPtr sub_commands_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace biped_driver_cpp

// ── main ─────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<biped_driver_cpp::CanBusNodeCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
