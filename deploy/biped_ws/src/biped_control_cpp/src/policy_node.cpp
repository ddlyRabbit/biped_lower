#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "biped_msgs/msg/mit_command.hpp"
#include "biped_msgs/msg/mit_command_array.hpp"

#include <yaml-cpp/yaml.h>
#include <onnxruntime_cxx_api.h>
#include "biped_control_cpp/obs_builder.hpp"

using namespace std::chrono_literals;
using namespace biped_control_cpp;

// Soft start timing
const double SOFT_START_POSITION_SECS = 2.0;
const double SOFT_START_GAIN_SECS = 1.0;
const double WALK_GAIN_RAMP_SECS = 0.1;

class PolicyNodeCpp : public rclcpp::Node {
public:
    PolicyNodeCpp() : Node("policy_node") {
        declare_parameter("onnx_model", "student_flat.onnx");
        declare_parameter("loop_rate", 50.0);
        declare_parameter("gains_file", "");
        declare_parameter("gain_scale", 1.0);

        declare_parameter("control_params_file", "");
        
        std::string model_path = get_parameter("onnx_model").as_string();
        rate_ = get_parameter("loop_rate").as_double();
        std::string gains_file = get_parameter("gains_file").as_string();
        gain_scale_ = get_parameter("gain_scale").as_double();

        std::string control_params_path = get_parameter("control_params_file").as_string();
        if (!control_params_path.empty()) {
            load_control_params(control_params_path);
        } else {
            load_control_params("");
        }

        gains_ = DEFAULT_GAINS;
        load_gains(gains_file);

        // Load ONNX model
        try {
            Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "PolicyNode");
            Ort::SessionOptions session_options;
            
            // Try enabling TensorRT / CUDA execution provider
            std::vector<std::string> available_providers = Ort::GetAvailableProviders();
            for (const auto& provider : available_providers) {
                if (provider == "TensorrtExecutionProvider") {
                    OrtTensorRTProviderOptions trt_options{};
                    session_options.AppendExecutionProvider_TensorRT(trt_options);
                    RCLCPP_INFO(get_logger(), "TensorRT Execution Provider enabled.");
                } else if (provider == "CUDAExecutionProvider") {
                    OrtCUDAProviderOptions cuda_options{};
                    session_options.AppendExecutionProvider_CUDA(cuda_options);
                    RCLCPP_INFO(get_logger(), "CUDA Execution Provider enabled.");
                }
            }

            session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
            
            // Get input/output info
            Ort::AllocatorWithDefaultOptions allocator;
            owned_input_names_.push_back(session_->GetInputNameAllocated(0, allocator));
            input_node_names_.push_back(owned_input_names_.back().get());

            owned_output_names_.push_back(session_->GetOutputNameAllocated(0, allocator));
            output_node_names_.push_back(owned_output_names_.back().get());

            RCLCPP_INFO(get_logger(), "ONNX model loaded: %s", model_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load ONNX model: %s", e.what());
        }

        auto sensor_qos = rclcpp::QoS(1).best_effort();

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos, std::bind(&PolicyNodeCpp::imu_cb, this, std::placeholders::_1));
        sub_gravity_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/imu/gravity", sensor_qos, std::bind(&PolicyNodeCpp::gravity_cb, this, std::placeholders::_1));
        sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", sensor_qos, std::bind(&PolicyNodeCpp::joint_cb, this, std::placeholders::_1));
        sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PolicyNodeCpp::cmd_cb, this, std::placeholders::_1));
        sub_fsm_ = create_subscription<std_msgs::msg::String>(
            "/state_machine", 10, std::bind(&PolicyNodeCpp::fsm_cb, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<biped_msgs::msg::MITCommandArray>("/joint_commands", 10);
        pub_viz_ = create_publisher<biped_msgs::msg::MITCommandArray>("/policy_viz", 10);
        pub_viz_js_ = create_publisher<sensor_msgs::msg::JointState>("/policy_viz_joints", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_),
            std::bind(&PolicyNodeCpp::loop_cb, this)
        );

        RCLCPP_INFO(get_logger(), "Policy node started (C++) — %.1fHz, gain_scale=%.2f", rate_, gain_scale_);
    }

private:
    double rate_, gain_scale_;
    std::unordered_map<std::string, std::pair<double, double>> gains_;
    ObsBuilder obs_builder_;

    std::array<float, 3> gyro_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gravity_ = {0.0f, 0.0f, 9.81f};
    std::array<float, 3> cmd_vel_ = {0.0f, 0.0f, 0.0f};
    std::unordered_map<std::string, double> joint_positions_;
    std::unordered_map<std::string, double> joint_velocities_;
    
    std::string fsm_state_ = "IDLE";
    double walk_start_time_ = -1.0;
    int debug_timer_ = 0;

    std::unique_ptr<Ort::Session> session_;
    std::vector<Ort::AllocatedStringPtr> owned_input_names_;
    std::vector<Ort::AllocatedStringPtr> owned_output_names_;
    std::vector<const char*> input_node_names_;
    std::vector<const char*> output_node_names_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_gravity_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fsm_;

    rclcpp::Publisher<biped_msgs::msg::MITCommandArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<biped_msgs::msg::MITCommandArray>::SharedPtr pub_viz_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_viz_js_;
    rclcpp::TimerBase::SharedPtr timer_;

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
            RCLCPP_INFO(get_logger(), "Loaded gains from %s", path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to load gains: %s, using defaults", e.what());
        }
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
        gyro_[0] = msg->angular_velocity.x;
        gyro_[1] = msg->angular_velocity.y;
        gyro_[2] = msg->angular_velocity.z;
    }

    void gravity_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        gravity_[0] = msg->vector.x;
        gravity_[1] = msg->vector.y;
        gravity_[2] = msg->vector.z;
    }

    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size()) joint_positions_[msg->name[i]] = msg->position[i];
            if (i < msg->velocity.size()) joint_velocities_[msg->name[i]] = msg->velocity[i];
        }
    }

    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_[0] = msg->linear.x;
        cmd_vel_[1] = msg->linear.y;
        cmd_vel_[2] = msg->angular.z;
    }

    void fsm_cb(const std_msgs::msg::String::SharedPtr msg) {
        std::string new_state = msg->data;
        for (auto& c : new_state) c = std::toupper(c);
        
        if (new_state != fsm_state_) {
            RCLCPP_INFO(get_logger(), "FSM: %s -> %s", fsm_state_.c_str(), new_state.c_str());
            if (new_state == "WALK" || new_state == "SIM_WALK") {
                walk_start_time_ = now().seconds();
            }
            fsm_state_ = new_state;
        }
    }

    void loop_cb() {
        if (!session_) return;
        if (fsm_state_ != "WALK" && fsm_state_ != "SIM_WALK") return;
        if (joint_positions_.empty()) return;

        auto obs_array = obs_builder_.build(gyro_, gravity_, cmd_vel_, joint_positions_, joint_velocities_);
        // RCLCPP_INFO(get_logger(), "[RAW] cmd_vel=%.2f,%.2f,%.2f", cmd_vel_[0], cmd_vel_[1], cmd_vel_[2]);
        // RCLCPP_INFO(get_logger(), "[RAW] gyro=%.2f,%.2f,%.2f", gyro_[0], gyro_[1], gyro_[2]);
        // RCLCPP_INFO(get_logger(), "[RAW] gravity=%.2f,%.2f,%.2f", gravity_[0], gravity_[1], gravity_[2]);
        // RCLCPP_INFO(get_logger(), "[RAW] gravity=%.2f,%.2f,%.2f", gravity_[0], gravity_[1], gravity_[2]);
        RCLCPP_INFO(get_logger(), "[RAW] obs_array=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", obs_array[0],obs_array[1],obs_array[2],obs_array[3],
        obs_array[4],obs_array[5],obs_array[6],obs_array[7],obs_array[8],obs_array[9]*57.3,
        obs_array[10]*57.3,obs_array[11]*57.3,obs_array[12]*57.3,obs_array[13]*57.3,obs_array[14]*57.3,obs_array[15]*57.3,
        obs_array[16]*57.3,obs_array[17]*57.3,obs_array[18]*57.3,obs_array[19]*57.3,obs_array[20]*57.3,obs_array[21],
        obs_array[22],obs_array[23],obs_array[24],obs_array[25],obs_array[26],obs_array[27],
        obs_array[28],obs_array[29],obs_array[30],obs_array[31],obs_array[32],obs_array[33],
        obs_array[34],obs_array[35],obs_array[36],obs_array[37],obs_array[38],obs_array[39],
        obs_array[40],obs_array[41],obs_array[42],obs_array[43],obs_array[44]);
        // Run ONNX Inference
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> input_shape = {1, 45};
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, obs_array.data(), obs_array.size(), input_shape.data(), input_shape.size());

        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr}, 
            input_node_names_.data(), 
            &input_tensor, 1, 
            output_node_names_.data(), 1);

        float* out_arr = output_tensors.front().GetTensorMutableData<float>();
        std::array<float, 12> actions;
        for (int i = 0; i < 12; i++) {
            actions[i] = std::max(-1.0f, std::min(1.0f, out_arr[i]));
        }
        RCLCPP_INFO(get_logger(), "[RAW] actions=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", actions[0], actions[1], actions[2], actions[3], actions[4], actions[5], actions[6], actions[7], actions[8], actions[9], actions[10], actions[11]);
        obs_builder_.update_last_action(actions);

        debug_timer_++;
        int debug_freq = static_cast<int>(rate_) * 10;
        if (debug_timer_ % debug_freq == 1) {
            RCLCPP_INFO(get_logger(), "[RAW] cmd_vel=%.2f,%.2f,%.2f", cmd_vel_[0], cmd_vel_[1], cmd_vel_[2]);
        }

        auto targets = ObsBuilder::action_to_positions(actions);

        biped_msgs::msg::MITCommandArray cmd_msg;
        cmd_msg.header.stamp = now();

        double walk_ramp = 1.0;
        if (walk_start_time_ > 0.0) {
            double walk_elapsed = now().seconds() - walk_start_time_;
            walk_ramp = std::min(1.0, 0.1 + 0.9 * (walk_elapsed / WALK_GAIN_RAMP_SECS));
        }

        double gs = get_parameter("gain_scale").as_double();

        for (const auto& name : JOINT_ORDER) {
            auto kp_kd = gains_.at(name);
            biped_msgs::msg::MITCommand cmd;
            cmd.joint_name = name;
            cmd.position = targets.at(name);
            cmd.velocity = 0.0;
            cmd.kp = kp_kd.first * gs * walk_ramp;
            cmd.kd = kp_kd.second * gs * walk_ramp;
            cmd.torque_ff = 0.0;
            cmd_msg.commands.push_back(cmd);
        }

        if (fsm_state_ == "SIM_WALK") {
            pub_viz_->publish(cmd_msg);
            sensor_msgs::msg::JointState js;
            js.header = cmd_msg.header;
            for (const auto& cmd : cmd_msg.commands) {
                js.name.push_back(cmd.joint_name);
                js.position.push_back(cmd.position);
                js.velocity.push_back(0.0);
                js.effort.push_back(0.0);
            }
            pub_viz_js_->publish(js);
        } else if (fsm_state_ == "WALK") {
            pub_cmd_->publish(cmd_msg);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolicyNodeCpp>());
    rclcpp::shutdown();
    return 0;
}
