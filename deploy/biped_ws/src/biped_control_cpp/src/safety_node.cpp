#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "biped_msgs/msg/mit_command_array.hpp"
#include "biped_msgs/msg/motor_state_array.hpp"

using namespace std::chrono_literals;

class SafetyNodeCpp : public rclcpp::Node {
public:
    SafetyNodeCpp() : Node("safety_node") {
        declare_parameter("max_pitch_deg", 45.0);
        declare_parameter("max_roll_deg", 30.0);
        declare_parameter("max_motor_temp", 80.0);
        declare_parameter("command_timeout_ms", 200.0);
        declare_parameter("imu_timeout_ms", 200.0);
        declare_parameter("check_rate", 50.0);

        double max_pitch_deg = get_parameter("max_pitch_deg").as_double();
        double max_roll_deg = get_parameter("max_roll_deg").as_double();
        max_pitch_ = max_pitch_deg * M_PI / 180.0;
        max_roll_ = max_roll_deg * M_PI / 180.0;
        max_temp_ = get_parameter("max_motor_temp").as_double();
        cmd_timeout_ = get_parameter("command_timeout_ms").as_double() / 1000.0;
        imu_timeout_ = get_parameter("imu_timeout_ms").as_double() / 1000.0;
        double rate = get_parameter("check_rate").as_double();

        auto sensor_qos = rclcpp::QoS(1).best_effort();

        sub_gravity_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/imu/gravity", sensor_qos, std::bind(&SafetyNodeCpp::gravity_cb, this, std::placeholders::_1));
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos, std::bind(&SafetyNodeCpp::imu_alive_cb, this, std::placeholders::_1));
        sub_motors_ = create_subscription<biped_msgs::msg::MotorStateArray>(
            "/motor_states", sensor_qos, std::bind(&SafetyNodeCpp::motor_cb, this, std::placeholders::_1));
        sub_cmds_ = create_subscription<biped_msgs::msg::MITCommandArray>(
            "/joint_commands", 10, std::bind(&SafetyNodeCpp::cmd_alive_cb, this, std::placeholders::_1));

        pub_status_ = create_publisher<std_msgs::msg::Bool>("/safety/status", 10);
        pub_fault_ = create_publisher<std_msgs::msg::String>("/safety/fault", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&SafetyNodeCpp::check_loop, this)
        );

        last_cmd_time_ = now().seconds();
        last_imu_time_ = now().seconds();

        RCLCPP_INFO(get_logger(), "Safety node started (C++) — pitch<%.0f°, roll<%.0f°, temp<%.0f°C",
                    max_pitch_deg, max_roll_deg, max_temp_);
    }

private:
    double max_pitch_, max_roll_, max_temp_, cmd_timeout_, imu_timeout_;
    
    double gravity_[3] = {0.0, 0.0, 9.81};
    bool gravity_received_ = false;
    std::unordered_map<std::string, float> motor_temps_;
    std::unordered_map<std::string, uint8_t> motor_faults_;
    
    double last_cmd_time_ = 0.0;
    double last_imu_time_ = 0.0;
    bool safety_ok_ = true;
    std::string fault_reason_ = "";

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_gravity_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<biped_msgs::msg::MotorStateArray>::SharedPtr sub_motors_;
    rclcpp::Subscription<biped_msgs::msg::MITCommandArray>::SharedPtr sub_cmds_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fault_;
    rclcpp::TimerBase::SharedPtr timer_;

    void gravity_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        gravity_[0] = msg->vector.x;
        gravity_[1] = msg->vector.y;
        gravity_[2] = msg->vector.z;
        gravity_received_ = true;
    }

    void imu_alive_cb(const sensor_msgs::msg::Imu::SharedPtr) {
        last_imu_time_ = now().seconds();
    }

    void motor_cb(const biped_msgs::msg::MotorStateArray::SharedPtr msg) {
        for (const auto& m : msg->motors) {
            motor_temps_[m.joint_name] = m.temperature;
            motor_faults_[m.joint_name] = m.fault_code;
        }
    }

    void cmd_alive_cb(const biped_msgs::msg::MITCommandArray::SharedPtr) {
        last_cmd_time_ = now().seconds();
    }

    void check_loop() {
        double current_time = now().seconds();
        bool ok = true;
        std::string reason = "";

        // 1. IMU orientation
        double g_norm = std::sqrt(gravity_[0]*gravity_[0] + gravity_[1]*gravity_[1] + gravity_[2]*gravity_[2]);
        if (gravity_received_ && g_norm > 0.1) {
            double g_unit[3] = {gravity_[0]/g_norm, gravity_[1]/g_norm, gravity_[2]/g_norm};
            double pitch = std::atan2(g_unit[0], g_unit[2]);
            double roll = std::atan2(g_unit[1], g_unit[2]);

            if (std::abs(pitch) > max_pitch_) {
                ok = false;
                char buf[64];
                snprintf(buf, sizeof(buf), "Pitch %.1f° exceeds limit", pitch * 180.0 / M_PI);
                reason = buf;
            } else if (std::abs(roll) > max_roll_) {
                ok = false;
                char buf[64];
                snprintf(buf, sizeof(buf), "Roll %.1f° exceeds limit", roll * 180.0 / M_PI);
                reason = buf;
            }
        }

        // 2. Motor temperatures
        if (ok) {
            for (const auto& [name, temp] : motor_temps_) {
                if (temp > max_temp_) {
                    ok = false;
                    char buf[64];
                    snprintf(buf, sizeof(buf), "%s temp %.1f°C exceeds limit", name.c_str(), temp);
                    reason = buf;
                    break;
                }
            }
        }

        // 3. Motor faults
        if (ok) {
            for (const auto& [name, fault] : motor_faults_) {
                if (fault > 0) {
                    ok = false;
                    reason = name + " fault code: " + std::to_string(fault);
                    break;
                }
            }
        }

        // 4. Command watchdog
        if (ok) {
            if (last_cmd_time_ > 0 && (current_time - last_cmd_time_) > cmd_timeout_) {
                ok = false;
                char buf[64];
                snprintf(buf, sizeof(buf), "Command timeout (%.0fms)", (current_time - last_cmd_time_) * 1000.0);
                reason = buf;
            }
        }

        // 5. IMU watchdog
        if (ok) {
            if ((current_time - last_imu_time_) > imu_timeout_) {
                ok = false;
                reason = "IMU data timeout";
            }
        }

        // Publish
        std_msgs::msg::Bool status_msg;
        status_msg.data = ok;
        pub_status_->publish(status_msg);

        if (!ok && (ok != safety_ok_ || reason != fault_reason_)) {
            std_msgs::msg::String fault_msg;
            fault_msg.data = reason;
            pub_fault_->publish(fault_msg);
            RCLCPP_ERROR(get_logger(), "SAFETY FAULT: %s", reason.c_str());
        }

        safety_ok_ = ok;
        fault_reason_ = reason;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNodeCpp>());
    rclcpp::shutdown();
    return 0;
}
