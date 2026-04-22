/**
 * @file unified_node.cpp
 * @brief Unified sense-and-control node — single-process IMU + CAN + policy.
 *
 * Replaces: can_bus_node_cpp + imu_node + policy_node_cpp
 *
 * Loop at 50Hz:
 *   1. READ  — CAN motor feedback (2 threads) + IMU I2C (main thread)
 *   2. THINK — Build observation (45d) + ONNX inference (12d actions)
 *   3. WRITE — Send MIT commands to motors (immediate)
 *   4. PUBLISH — ROS messages for Foxglove / safety / state machine
 *
 * FSM states handled:
 *   IDLE      → read motors, send zero-torque
 *   STAND     → soft ramp to default positions (same math as state_machine_node)
 *   WALK      → policy output → motors
 *   SIM_WALK  → policy output → /policy_viz only, motors hold STAND
 *
 * Safety, state machine, and teleop nodes remain separate.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <biped_msgs/msg/mit_command.hpp>
#include <biped_msgs/msg/mit_command_array.hpp>
#include <biped_msgs/msg/motor_state.hpp>
#include <biped_msgs/msg/motor_state_array.hpp>

#include <yaml-cpp/yaml.h>
#include <onnxruntime_cxx_api.h>

#include "biped_driver_cpp/motor_manager.hpp"
#include "biped_driver_cpp/ankle_linkage.hpp"
#include "biped_driver_cpp/bno085_reader.hpp"
#include "biped_driver_cpp/im10a_reader.hpp"
#include "biped_control_cpp/obs_builder.hpp"

using namespace biped_driver_cpp;
using namespace biped_control_cpp;

// ── URDF joint-space limits for ankle (same as can_bus_node) ──────
static constexpr double ANKLE_PITCH_LO = -0.87267;
static constexpr double ANKLE_PITCH_HI =  0.52360;
static constexpr double ANKLE_ROLL_LO  = -0.26180;
static constexpr double ANKLE_ROLL_HI  =  0.26180;

// ── Soft-start / ramp timings ────────────────────────────────────
static constexpr double STAND_RAMP_SECS      = 2.0;
static constexpr double STAND_GAIN_RAMP_SECS = 1.0;
static constexpr double WALK_GAIN_RAMP_SECS  = 0.1;

// ── Feedback snapshot ────────────────────────────────────────────

struct MotorFB {
    double position = 0.0;
    double velocity = 0.0;
    double torque   = 0.0;
    double temp     = 0.0;
    int fault       = 0;
    int mode        = 0;
    bool valid      = false;
};

// ── CAN read worker (one per bus) ────────────────────────────────

class CanReadWorker {
public:
    CanReadWorker(const std::string& bus_name,
                  BipedMotorManager& mgr,
                  const std::vector<std::string>& motor_names)
        : bus_name_(bus_name), mgr_(mgr), motor_names_(motor_names) {
        for (auto& n : motor_names) {
            fb_[n] = MotorFB{};
            prev_pos_[n] = 0.0;
        }
    }

    /// Read all motors on this bus. Called from its own thread.
    void read_all(double timeout = 0.005) {
        for (auto& name : motor_names_) {
            try {
                auto fb = mgr_.read_feedback(name, timeout);
                if (fb) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    prev_pos_[name] = fb->position;
                    fb_[name] = MotorFB{
                        fb->position, fb->velocity, fb->torque,
                        fb->temperature, fb->fault_code, fb->mode_status, true
                    };
                }
            } catch (...) {
                // Motor read errors are non-fatal per-cycle
            }
        }
    }

    /// Get latest feedback snapshot (thread-safe).
    std::unordered_map<std::string, MotorFB> snapshot() {
        std::lock_guard<std::mutex> lk(mtx_);
        return fb_;
    }

    std::optional<double> last_pos(const std::string& name) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = prev_pos_.find(name);
        if (it != prev_pos_.end()) return it->second;
        return std::nullopt;
    }

private:
    std::string bus_name_;
    BipedMotorManager& mgr_;
    std::vector<std::string> motor_names_;
    std::mutex mtx_;
    std::unordered_map<std::string, MotorFB> fb_;
    std::unordered_map<std::string, double> prev_pos_;
};

// ── Unified Node ─────────────────────────────────────────────────

class UnifiedNode : public rclcpp::Node {
public:
    UnifiedNode() : Node("unified_node") {
        // ── Parameters ───────────────────────────────────────────
        declare_parameter("robot_config", "");
        declare_parameter("calibration_file", "");
        declare_parameter("onnx_model", "student_flat.onnx");
        declare_parameter("gains_file", "");
        declare_parameter("gain_scale", 1.0);
        declare_parameter("loop_rate", 50.0);
        declare_parameter("imu_type", "bno085");
        declare_parameter("im10a_port", "/dev/ttyUSB0");
        declare_parameter("im10a_baud", 460800);
        declare_parameter("i2c_bus", 1);
        declare_parameter("i2c_address", 0x4B);
        declare_parameter("imu_rate_hz", 200.0);
        declare_parameter("use_game_quaternion", false);
        declare_parameter("imu_reset_pin", 4);

        std::string robot_cfg_path = get_parameter("robot_config").as_string();
        std::string cal_path       = get_parameter("calibration_file").as_string();
        std::string model_path     = get_parameter("onnx_model").as_string();
        std::string gains_path     = get_parameter("gains_file").as_string();
        gain_scale_                = get_parameter("gain_scale").as_double();
        loop_rate_                 = get_parameter("loop_rate").as_double();

        // ── Load robot config ────────────────────────────────────
        YAML::Node robot_yaml, cal_yaml;
        if (robot_cfg_path.empty()) {
            RCLCPP_ERROR(get_logger(), "robot_config parameter required");
            return;
        }
        robot_yaml = YAML::LoadFile(robot_cfg_path);
        if (!cal_path.empty()) {
            try { cal_yaml = YAML::LoadFile(cal_path); }
            catch (...) { RCLCPP_WARN(get_logger(), "No calibration loaded"); }
        }

        // ── Init motors ──────────────────────────────────────────
        mgr_ = std::make_unique<BipedMotorManager>(
            build_manager_from_yaml(robot_yaml, cal_yaml));

        // Group motors by bus
        std::unordered_map<std::string, std::vector<std::string>> bus_motors;
        for (auto& [name, jcfg] : mgr_->joints()) {
            bus_motors[jcfg.can_bus].push_back(name);
        }

        for (auto& [bus_name, motors] : bus_motors) {
            can_workers_[bus_name] = std::make_unique<CanReadWorker>(bus_name, *mgr_, motors);
        }

        // ── Connect motors ──────────────────────────────────────
        mgr_->connect_all();
        mgr_->flush_all();
        mgr_->enable_all();
        RCLCPP_INFO(get_logger(), "All motors enabled in MIT mode.");

        // ── Init IMU ─────────────────────────────────────────────
        imu_type_ = get_parameter("imu_type").as_string();

        if (imu_type_ == "im10a") {
            std::string port = get_parameter("im10a_port").as_string();
            int baud = get_parameter("im10a_baud").as_int();
            if (im10a_imu_.init(port, baud)) {
                RCLCPP_INFO(get_logger(), "IM10A initialized — port %s, baud %d", port.c_str(), baud);
            } else {
                RCLCPP_FATAL(get_logger(), "IM10A init failed! Aborting.");
                throw std::runtime_error("IM10A init failed");
            }
        } else {
            int i2c_bus      = get_parameter("i2c_bus").as_int();
            int i2c_addr     = get_parameter("i2c_address").as_int();
            double imu_rate  = get_parameter("imu_rate_hz").as_double();
            bool game_quat   = get_parameter("use_game_quaternion").as_bool();
            int reset_pin    = get_parameter("imu_reset_pin").as_int();

            if (bno085_imu_.init(i2c_bus, i2c_addr, reset_pin, imu_rate, game_quat)) {
                RCLCPP_INFO(get_logger(), "BNO085 initialized — bus %d, addr 0x%02X, %.0fHz",
                            i2c_bus, i2c_addr, imu_rate);
            } else {
                RCLCPP_FATAL(get_logger(), "BNO085 init failed! Aborting.");
                throw std::runtime_error("BNO085 init failed");
            }
        }

        // ── Init ONNX ───────────────────────────────────────────
        gains_ = DEFAULT_GAINS;
        load_gains(gains_path);
        init_onnx(model_path);

        // ── Init state ───────────────────────────────────────────
        for (auto& name : JOINT_ORDER) {
            stand_start_pos_[name] = DEFAULT_POSITIONS.at(name);
        }

        // ── ROS pub/sub ──────────────────────────────────────────
        auto sensor_qos = rclcpp::QoS(1).best_effort();

        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_vel_[0] = msg->linear.x;
                cmd_vel_[1] = msg->linear.y;
                cmd_vel_[2] = msg->angular.z;
            });

        sub_fsm_ = create_subscription<std_msgs::msg::String>(
            "/state_machine", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::string s = msg->data;
                for (auto& c : s) c = std::toupper(c);
                if (s != fsm_state_) {
                    RCLCPP_INFO(get_logger(), "FSM: %s -> %s", fsm_state_.c_str(), s.c_str());
                    if (s == "WALK" || s == "SIM_WALK") {
                        walk_start_time_ = now().seconds();
                    }
                    if (s == "STAND") {
                        stand_start_time_ = now().seconds();
                        stand_start_pos_ = latest_joint_positions();
                    }
                    fsm_state_ = s;
                }
            });

        pub_joints_    = create_publisher<sensor_msgs::msg::JointState>("/joint_states", sensor_qos);
        pub_motors_    = create_publisher<biped_msgs::msg::MotorStateArray>("/motor_states", sensor_qos);
        pub_imu_       = create_publisher<sensor_msgs::msg::Imu>("/imu/data", sensor_qos);
        pub_grav_      = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/gravity", sensor_qos);
        pub_commands_  = create_publisher<biped_msgs::msg::MITCommandArray>("/joint_commands", sensor_qos);
        pub_viz_       = create_publisher<biped_msgs::msg::MITCommandArray>("/policy_viz", sensor_qos);
        pub_viz_js_    = create_publisher<sensor_msgs::msg::JointState>("/policy_viz_joints", sensor_qos);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ── Control timer ────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / loop_rate_),
            [this]() { control_loop(); });

        RCLCPP_INFO(get_logger(), "Unified node ready — %.0fHz, gain_scale=%.2f", loop_rate_, gain_scale_);
    }

    ~UnifiedNode() override {
        RCLCPP_INFO(get_logger(), "Shutting down unified node");
        try { mgr_->disable_all(); } catch (...) {}
        mgr_->disconnect_all();
    }

private:
    // ── Members ───────────────────────────────────────────────────
    std::unique_ptr<BipedMotorManager> mgr_;
    std::string imu_type_;
    biped_driver_cpp::Bno085Reader bno085_imu_;
    biped_driver_cpp::Im10aReader im10a_imu_;
    ObsBuilder obs_builder_;

    std::unordered_map<std::string, std::unique_ptr<CanReadWorker>> can_workers_;
    std::unordered_map<std::string, std::pair<double, double>> gains_;
    double gain_scale_ = 1.0;
    double loop_rate_  = 50.0;

    // ONNX
    std::unique_ptr<Ort::Session> session_;
    std::vector<Ort::AllocatedStringPtr> owned_in_names_;
    std::vector<Ort::AllocatedStringPtr> owned_out_names_;
    std::vector<const char*> in_names_;
    std::vector<const char*> out_names_;

    // State
    std::string fsm_state_ = "IDLE";
    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
    double walk_start_time_  = -1.0;
    double stand_start_time_ = 0.0;
    std::unordered_map<std::string, double> stand_start_pos_;

    // Last sent motor commands — re-sent in Phase 1 to prime fresh feedback
    biped_msgs::msg::MITCommandArray last_sent_cmd_;
    bool has_last_cmd_ = false;

    // Latest sensor data (written by main loop only — no contention)
    std::unordered_map<std::string, double> joint_pos_;
    std::unordered_map<std::string, double> joint_vel_;
    std::unordered_map<std::string, MotorFB> motor_fb_raw_;  // motor-space
    float gyro_[3]     = {0.0f, 0.0f, 0.0f};
    float gravity_[3]  = {0.0f, 0.0f, -9.81f};
    float quat_[4]     = {0.0f, 0.0f, 0.0f, 1.0f};

    // Diagnostics
    int debug_counter_ = 0;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fsm_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
    rclcpp::Publisher<biped_msgs::msg::MotorStateArray>::SharedPtr pub_motors_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_grav_;
    rclcpp::Publisher<biped_msgs::msg::MITCommandArray>::SharedPtr pub_commands_;
    rclcpp::Publisher<biped_msgs::msg::MITCommandArray>::SharedPtr pub_viz_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_viz_js_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── ONNX init ────────────────────────────────────────────────
    void init_onnx(const std::string& model_path) {
        try {
            Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "UnifiedNode");
            Ort::SessionOptions opts;

            // Try CUDA/TensorRT (won't hurt if unavailable, e.g. on Pi)
            for (auto& provider : Ort::GetAvailableProviders()) {
                if (provider == "TensorrtExecutionProvider") {
                    OrtTensorRTProviderOptions trt_opts{};
                    opts.AppendExecutionProvider_TensorRT(trt_opts);
                } else if (provider == "CUDAExecutionProvider") {
                    OrtCUDAProviderOptions cuda_opts{};
                    opts.AppendExecutionProvider_CUDA(cuda_opts);
                }
            }

            session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), opts);

            Ort::AllocatorWithDefaultOptions alloc;
            owned_in_names_.push_back(session_->GetInputNameAllocated(0, alloc));
            in_names_.push_back(owned_in_names_.back().get());
            owned_out_names_.push_back(session_->GetOutputNameAllocated(0, alloc));
            out_names_.push_back(owned_out_names_.back().get());

            RCLCPP_INFO(get_logger(), "ONNX loaded: %s", model_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "ONNX load failed: %s", e.what());
        }
    }

    void load_gains(const std::string& path) {
        if (path.empty()) return;
        try {
            YAML::Node data = YAML::LoadFile(path);
            for (const auto& name : JOINT_ORDER) {
                if (data[name]) {
                    double kp = data[name]["kp"] ? data[name]["kp"].as<double>() : gains_[name].first;
                    double kd = data[name]["kd"] ? data[name]["kd"].as<double>() : gains_[name].second;
                    gains_[name] = {kp, kd};
                }
            }
        } catch (...) {}
    }

    // ── Helpers ───────────────────────────────────────────────────

    std::unordered_map<std::string, double> latest_joint_positions() {
        return joint_pos_;
    }

    // ── Main control loop ─────────────────────────────────────────
    void control_loop() {
        auto t0 = std::chrono::steady_clock::now();

        // ============================================================
        // PHASE 1: RE-SEND last cycle's commands to prime motors
        // ============================================================
        // This ensures the motors respond with fresh state that we'll
        // read in Phase 2. On first cycle there are no commands yet.
        if (has_last_cmd_) {
            send_motor_commands(last_sent_cmd_);
        }

        // ============================================================
        // PHASE 2: READ fresh sensors
        // ============================================================

        // Read CAN feedback — parallel per bus
        std::vector<std::thread> readers;
        for (auto& [bus, worker] : can_workers_) {
            CanReadWorker* w = worker.get();
            readers.emplace_back([w]() { w->read_all(0.005); });
        }

        // Read IMU on main thread while CAN threads work
        biped_driver_cpp::ImuData imu_data;
        if (imu_type_ == "im10a") {
            imu_data = im10a_imu_.read();
        } else {
            imu_data = bno085_imu_.read();
        }
        if (imu_data.valid) {
            quat_[0] = imu_data.quaternion[0];
            quat_[1] = imu_data.quaternion[1];
            quat_[2] = imu_data.quaternion[2];
            quat_[3] = imu_data.quaternion[3];
            gyro_[0] = imu_data.gyro[0];
            gyro_[1] = imu_data.gyro[1];
            gyro_[2] = imu_data.gyro[2];
            gravity_[0] = imu_data.gravity[0];
            gravity_[1] = imu_data.gravity[1];
            gravity_[2] = imu_data.gravity[2];
        }

        // Wait for CAN reads
        for (auto& t : readers) t.join();

        // Merge feedback into joint-space
        motor_fb_raw_.clear();
        joint_pos_.clear();
        joint_vel_.clear();

        std::unordered_set<std::string> processed;
        for (auto& [bus, worker] : can_workers_) {
            auto fb_map = worker->snapshot();
            for (auto& [name, fb] : fb_map) {
                motor_fb_raw_[name] = fb;

                auto pair = mgr_->get_ankle_pair(name);
                if (pair && !processed.count(pair->first)) {
                    auto& [top, bot] = *pair;
                    auto fb_top = fb_map.count(top) ? fb_map.at(top) : MotorFB{};
                    auto fb_bot = fb_map.count(bot) ? fb_map.at(bot) : MotorFB{};

                    if (fb_top.valid && fb_bot.valid) {
                        int psign = (top[0] == 'L') ? -1 : 1;
                        auto [fp, fr] = ankle_motors_to_feedback(fb_top.position, fb_bot.position, psign);
                        auto [fpv, frv] = ankle_motors_to_feedback(fb_top.velocity, fb_bot.velocity, psign);

                        auto& amj = ankle_motor_to_joint();
                        joint_pos_[amj.at(top)]    = fp;
                        joint_vel_[amj.at(top)]    = fpv;
                        joint_pos_[amj.at(bot)]    = fr;
                        joint_vel_[amj.at(bot)]    = frv;
                    }
                    processed.insert(top);
                    processed.insert(bot);
                } else if (!pair && !is_ankle_motor(name)) {
                    joint_pos_[name] = fb.position;
                    joint_vel_[name] = fb.velocity;
                }
            }
        }

        // ============================================================
        // PHASE 3: FSM-dependent think + SEND new commands
        // ============================================================
        // New commands are sent to motors IMMEDIATELY for lowest latency.
        // They are also stored for Phase 1 re-send next cycle.

        if (fsm_state_ == "WALK") {
            do_walk(false);
        } else if (fsm_state_ == "SIM_WALK") {
            do_walk(true);
        } else if (fsm_state_ == "STAND") {
            do_stand();
        } else {
            do_idle();
        }

        // ============================================================
        // PHASE 4: PUBLISH ROS messages
        // ============================================================
        publish_ros();

        // Diagnostics
        debug_counter_++;
        if (debug_counter_ % static_cast<int>(loop_rate_) == 0) {
            auto dt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            RCLCPP_INFO(get_logger(), "Loop: %.1fms (%.0f%% of %.0fms budget) | %s",
                        dt * 1000.0, dt * loop_rate_ * 100.0, 1000.0 / loop_rate_,
                        fsm_state_.c_str());
        }
    }

    // ── WALK: policy inference → motors ───────────────────────────
    void do_walk(bool sim_only) {
        if (!session_) return;
        if (joint_pos_.empty()) return;

        // Build observation
        std::array<float, 3> gyro_arr  = {gyro_[0], gyro_[1], gyro_[2]};
        std::array<float, 3> grav_arr  = {gravity_[0], gravity_[1], gravity_[2]};
        std::array<float, 3> cmd_arr   = cmd_vel_;

        auto obs = obs_builder_.build(gyro_arr, grav_arr, cmd_arr, joint_pos_, joint_vel_);

        // ONNX inference
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> shape = {1, 45};
        auto input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, obs.data(), obs.size(), shape.data(), shape.size());

        auto output = session_->Run(
            Ort::RunOptions{nullptr},
            in_names_.data(), &input_tensor, 1,
            out_names_.data(), 1);

        float* out = output.front().GetTensorMutableData<float>();
        std::array<float, 12> actions;
        for (int i = 0; i < 12; ++i) {
            actions[i] = std::max(-1.0f, std::min(1.0f, out[i]));
        }
        obs_builder_.update_last_action(actions);

        auto targets = ObsBuilder::action_to_positions(actions);

        // Gain ramp after transition to WALK
        double walk_ramp = 1.0;
        if (walk_start_time_ > 0.0) {
            double elapsed = now().seconds() - walk_start_time_;
            walk_ramp = std::min(1.0, 0.1 + 0.9 * (elapsed / WALK_GAIN_RAMP_SECS));
        }
        double gs = get_parameter("gain_scale").as_double();

        // Build commands
        biped_msgs::msg::MITCommandArray cmd_msg;
        cmd_msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            auto kp_kd = gains_.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name  = name;
            cmd.position    = targets.at(name);
            cmd.velocity    = 0.0;
            cmd.kp          = kp_kd.first * gs * walk_ramp;
            cmd.kd          = kp_kd.second * gs * walk_ramp;
            cmd.torque_ff   = 0.0;
            cmd_msg.commands.push_back(cmd);
        }

        if (sim_only) {
            // SIM_WALK: publish viz, motors hold STAND
            pub_viz_->publish(cmd_msg);
            sensor_msgs::msg::JointState js;
            js.header = cmd_msg.header;
            for (auto& cmd : cmd_msg.commands) {
                js.name.push_back(cmd.joint_name);
                js.position.push_back(cmd.position);
                js.velocity.push_back(0.0);
                js.effort.push_back(0.0);
            }
            pub_viz_js_->publish(js);

            // Hold motors at STAND (store for Phase 1 re-send)
            auto stand_cmd = build_stand_commands();
            send_motor_commands(stand_cmd);
            last_sent_cmd_ = stand_cmd;
            has_last_cmd_ = true;
        } else {
            // WALK: send to motors immediately
            send_motor_commands(cmd_msg);
            last_sent_cmd_ = cmd_msg;
            has_last_cmd_ = true;
            pub_commands_->publish(cmd_msg);
        }
    }

    // ── STAND: soft ramp to default positions ─────────────────────
    void do_stand() {
        auto cmd_msg = build_stand_commands();
        send_motor_commands(cmd_msg);
        last_sent_cmd_ = cmd_msg;
        has_last_cmd_ = true;
        pub_commands_->publish(cmd_msg);
    }

    biped_msgs::msg::MITCommandArray build_stand_commands() {
        double elapsed = now().seconds() - stand_start_time_;
        double pos_alpha = std::min(elapsed / STAND_RAMP_SECS, 1.0);
        double gain_elapsed = std::max(0.0, elapsed - STAND_RAMP_SECS);
        double gain_alpha = std::min(gain_elapsed / STAND_GAIN_RAMP_SECS, 1.0);

        double kp_scale = 0.1 + 0.9 * gain_alpha;
        double kd_scale = 0.2 + 0.8 * gain_alpha;
        double gs = get_parameter("gain_scale").as_double();

        biped_msgs::msg::MITCommandArray cmd_msg;
        cmd_msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            double start = stand_start_pos_.count(name) ? stand_start_pos_[name] : DEFAULT_POSITIONS.at(name);
            double target = DEFAULT_POSITIONS.at(name);
            double pos = start + (target - start) * pos_alpha;

            auto kp_kd = gains_.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name  = name;
            cmd.position    = pos;
            cmd.velocity    = 0.0;
            cmd.kp          = kp_kd.first * kp_scale * gs;
            cmd.kd          = kp_kd.second * kd_scale * gs;
            cmd.torque_ff   = 0.0;
            cmd_msg.commands.push_back(cmd);
        }

        return cmd_msg;
    }

    // ── IDLE / ESTOP: zero torque ─────────────────────────────────
    void do_idle() {
        biped_msgs::msg::MITCommandArray cmd_msg;
        cmd_msg.header.stamp = now();

        for (const auto& name : JOINT_ORDER) {
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position   = 0.0;
            cmd.velocity   = 0.0;
            cmd.kp         = 0.0;
            cmd.kd         = 0.0;
            cmd.torque_ff  = 0.0;
            cmd_msg.commands.push_back(cmd);
        }

        send_motor_commands(cmd_msg);
        last_sent_cmd_ = cmd_msg;
        has_last_cmd_ = true;
        pub_commands_->publish(cmd_msg);
    }

    // ── Send MIT commands to CAN buses ────────────────────────────
    void send_motor_commands(const biped_msgs::msg::MITCommandArray& cmd_msg) {
        // Build lookup
        std::unordered_map<std::string, const biped_msgs::msg::MITCommand*> cmd_map;
        for (auto& cmd : cmd_msg.commands) {
            cmd_map[cmd.joint_name] = &cmd;
        }

        // Send per bus
        for (auto& [bus_name, worker] : can_workers_) {
            auto& bus_motors = can_worker_motor_names(bus_name);

            // Build ankle pair set for this bus
            std::unordered_set<std::string> seen;
            std::vector<std::string> normal;
            std::vector<std::pair<std::string, std::string>> ankle_pairs;

            for (auto& name : bus_motors) {
                if (seen.count(name)) continue;
                auto pair = mgr_->get_ankle_pair(name);
                if (pair) {
                    int found = 0;
                    for (auto& n : bus_motors) {
                        if (n == pair->first || n == pair->second) found++;
                    }
                    bool both_on_bus = (found == 2);
                    if (both_on_bus) {
                        ankle_pairs.push_back(*pair);
                        seen.insert(pair->first);
                        seen.insert(pair->second);
                    }
                } else {
                    normal.push_back(name);
                    seen.insert(name);
                }
            }

            // Send normal motors
            for (auto& name : normal) {
                auto it = cmd_map.find(name);
                if (it != cmd_map.end()) {
                    auto& c = *it->second;
                    mgr_->send_mit_command(name, c.position, c.kp, c.kd,
                                           c.velocity, c.torque_ff,
                                           worker->last_pos(name));
                } else {
                    mgr_->send_mit_command(name, 0.0, 0.0, 0.0);
                }
            }

            // Send ankle pairs
            for (auto& [top, bot] : ankle_pairs) {
                auto& amj = ankle_motor_to_joint();
                std::string pitch_joint = amj.at(top);
                std::string roll_joint  = amj.at(bot);
                int pitch_sign = (top[0] == 'L') ? -1 : 1;

                auto pit = cmd_map.find(pitch_joint);
                auto rit = cmd_map.find(roll_joint);

                if (pit != cmd_map.end() && rit != cmd_map.end()) {
                    double pp = std::max(ANKLE_PITCH_LO,
                               std::min(ANKLE_PITCH_HI, (double)pit->second->position));
                    double rp = std::max(ANKLE_ROLL_LO,
                                std::min(ANKLE_ROLL_HI, (double)rit->second->position));

                    auto [motor_upper, motor_lower] = ankle_command_to_motors(pp, rp, pitch_sign);

                    double kp  = (pit->second->kp + rit->second->kp) / 2.0;
                    double kd  = (pit->second->kd + rit->second->kd) / 2.0;
                    double tff = (pit->second->torque_ff + rit->second->torque_ff) / 2.0;

                    mgr_->send_ankle_mit_command(top, motor_upper, kp, kd, 0.0, tff,
                                                  worker->last_pos(top));
                    mgr_->send_ankle_mit_command(bot, motor_lower, kp, kd, 0.0, tff,
                                                  worker->last_pos(bot));
                } else {
                    mgr_->send_ankle_mit_command(top, 0.0, 0.0, 0.0);
                    mgr_->send_ankle_mit_command(bot, 0.0, 0.0, 0.0);
                }
            }
        }
    }

    /// Get motor names belonging to a specific CAN worker bus.
    /// Cached on first call per bus.
    std::vector<std::string>& can_worker_motor_names(const std::string& bus_name) {
        auto it = bus_motor_names_cache_.find(bus_name);
        if (it != bus_motor_names_cache_.end()) return it->second;
        auto& names = bus_motor_names_cache_[bus_name];
        for (auto& [name, jcfg] : mgr_->joints()) {
            if (jcfg.can_bus == bus_name) names.push_back(name);
        }
        return names;
    }

    std::unordered_map<std::string, std::vector<std::string>> bus_motor_names_cache_;

    // ── Publish ROS messages ──────────────────────────────────────
    void publish_ros() {
        auto stamp = now();

        // 1. /joint_states
        sensor_msgs::msg::JointState js;
        js.header.stamp = stamp;
        for (auto& name : JOINT_ORDER) {
            js.name.push_back(name);
            js.position.push_back(joint_pos_.count(name) ? joint_pos_[name] : 0.0);
            js.velocity.push_back(joint_vel_.count(name) ? joint_vel_[name] : 0.0);
            js.effort.push_back(0.0);
        }
        pub_joints_->publish(js);

        // 2. /motor_states
        biped_msgs::msg::MotorStateArray ms_arr;
        ms_arr.header.stamp = stamp;
        for (auto& [name, fb] : motor_fb_raw_) {
            biped_msgs::msg::MotorState ms;
            ms.joint_name  = name;
            auto jit = mgr_->joints().find(name);
            if (jit != mgr_->joints().end()) {
                ms.can_id = static_cast<uint8_t>(jit->second.can_id);
            }
            ms.position    = fb.position;
            ms.velocity    = fb.velocity;
            ms.torque      = fb.torque;
            ms.temperature = fb.temp;
            ms.fault_code  = fb.fault;
            ms.mode_status = fb.mode;
            ms_arr.motors.push_back(ms);
        }
        pub_motors_->publish(ms_arr);

        // 3. /imu/data
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = stamp;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.x = quat_[0];
        imu_msg.orientation.y = quat_[1];
        imu_msg.orientation.z = quat_[2];
        imu_msg.orientation.w = quat_[3];
        imu_msg.angular_velocity.x = gyro_[0];
        imu_msg.angular_velocity.y = gyro_[1];
        imu_msg.angular_velocity.z = gyro_[2];
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.angular_velocity_covariance[0] = -1.0;
        imu_msg.linear_acceleration_covariance[0] = -1.0;
        pub_imu_->publish(imu_msg);

        // 4. /imu/gravity
        geometry_msgs::msg::Vector3Stamped grav_msg;
        grav_msg.header.stamp    = stamp;
        grav_msg.header.frame_id = "imu_link";
        grav_msg.vector.x = gravity_[0];
        grav_msg.vector.y = gravity_[1];
        grav_msg.vector.z = gravity_[2];
        pub_grav_->publish(grav_msg);

        // 5. TF odom → base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.rotation.x = quat_[0];
        tf.transform.rotation.y = quat_[1];
        tf.transform.rotation.z = quat_[2];
        tf.transform.rotation.w = quat_[3];
        tf_broadcaster_->sendTransform(tf);
    }
};

// ── main ─────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedNode>());
    rclcpp::shutdown();
    return 0;
}
