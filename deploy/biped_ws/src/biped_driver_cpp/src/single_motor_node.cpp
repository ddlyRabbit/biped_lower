#include "rclcpp/rclcpp.hpp"
#include "biped_msgs/msg/motor_state.hpp"
#include "biped_msgs/msg/mit_command.hpp"
#include "biped_driver_cpp/robstride_bus.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <chrono>

using namespace std::chrono_literals;
using namespace biped_driver_cpp;

struct MotorConfig {
    std::string interface;
    int id;
    std::string model;
    std::string name;
};

std::optional<MotorConfig> parse_target_motor(const std::string& yaml_path, const std::string& target_name) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        for (auto bus_it = root.begin(); bus_it != root.end(); ++bus_it) {
            std::string bus_key = bus_it->first.as<std::string>();
            auto bus_cfg = bus_it->second;
            if (!bus_cfg.IsMap() || !bus_cfg["motors"]) continue;

            std::string interface = bus_cfg["interface"] ? bus_cfg["interface"].as<std::string>() : bus_key;
            auto motors_cfg = bus_cfg["motors"];
            for (auto m_it = motors_cfg.begin(); m_it != motors_cfg.end(); ++m_it) {
                std::string name = m_it->first.as<std::string>();
                if (name == target_name) {
                    auto mcfg = m_it->second;
                    MotorConfig cfg;
                    cfg.interface = interface;
                    cfg.id = mcfg["id"].as<int>();
                    cfg.model = mcfg["type"].as<std::string>();
                    cfg.name = name;
                    // normalize model string (e.g. RS04 -> rs-04)
                    if (cfg.model == "RS02" || cfg.model == "RS-02") cfg.model = "rs-02";
                    else if (cfg.model == "RS03" || cfg.model == "RS-03") cfg.model = "rs-03";
                    else if (cfg.model == "RS04" || cfg.model == "RS-04") cfg.model = "rs-04";
                    else if (cfg.model == "RS08" || cfg.model == "RS-08") cfg.model = "rs-08";
                    return cfg;
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("parser"), "Failed to parse YAML: %s", e.what());
    }
    return std::nullopt;
}

// Global state for sharing between background input thread and 200Hz control loop
std::atomic<bool> running{true};
std::atomic<bool> enabled{false};
std::atomic<bool> request_clear_fault{false};

std::atomic<double> active_pos{0.0};
std::atomic<double> active_kp{0.0};
std::atomic<double> active_kd{0.0};
std::atomic<double> active_torque{0.0};

std::atomic<double> staged_pos{0.0};
std::atomic<double> staged_kp{0.0};
std::atomic<double> staged_kd{0.0};
std::atomic<double> staged_torque{0.0};

std::atomic<double> fb_pos{0.0};
std::atomic<double> fb_vel{0.0};
std::atomic<double> fb_tau{0.0};
std::atomic<double> fb_temp{0.0};

std::string motor_name;
std::string motor_interface;
int motor_id;
std::string motor_model;

void draw_ui() {
    // Clear screen and move cursor to top-left
    std::cout << "\033[2J\033[1;1H";

    std::cout << "=== SINGLE MOTOR RAW CONTROL: " << motor_name << " ===\n";
    std::cout << "Status: [" << (enabled.load() ? "\033[1;32mENABLED\033[0m" : "\033[1;31mDISABLED\033[0m") << "]  |  Link: [" 
              << motor_interface << ", ID: " << motor_id << ", Type: " << motor_model << "]\n\n";

    std::cout << "--- MOTOR FEEDBACK (200 Hz) ---\n";
    std::cout << "Pos: " << std::showpos << fb_pos.load() << " rad | Vel: " 
              << fb_vel.load() << " rad/s | Torque: " 
              << fb_tau.load() << " Nm | Temp: " 
              << std::noshowpos << fb_temp.load() << "°C\n\n";

    std::cout << "--- CONTROL PARAMETERS ---\n";
    std::cout << "          [ACTIVE]      [STAGED] (Edit with keys)\n";
    
    printf("Pos    : %8.3f      %8.3f     (w/s)\n", active_pos.load(), staged_pos.load());
    printf("Kp     : %8.3f      %8.3f     (p/o)\n", active_kp.load(), staged_kp.load());
    printf("Kd     : %8.3f      %8.3f     (k/j)\n", active_kd.load(), staged_kd.load());
    printf("Torque : %8.3f      %8.3f     (t/g)\n\n", active_torque.load(), staged_torque.load());

    std::cout << "Controls:\n";
    std::cout << "  [e]/[d] Enable/Disable immediately\n";
    std::cout << "  [c]     Clear Fault (disables motor)\n";
    std::cout << "  [ENTER] Commit STAGED -> ACTIVE\n";
    std::cout << "  [q]     Quit safely\n";
    std::cout << std::flush;
}

#include <sys/select.h>

void input_thread() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running) {
        draw_ui();
        
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms timeout
        
        int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
        
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char c = getchar();
            if (c == 'q') {
                running = false;
            } else if (c == 'e') {
                enabled = true;
            } else if (c == 'd') {
            } else if (c == 'c') {
                request_clear_fault = true;
                enabled = false;
            } else if (c == '\n' || c == '\r') {
                active_pos = staged_pos.load();
                active_kp = staged_kp.load();
                active_kd = staged_kd.load();
                active_torque = staged_torque.load();
            } else if (c == 'w') { staged_pos = staged_pos.load() + 0.05; }
            else if (c == 's') { staged_pos = staged_pos.load() - 0.05; }
            else if (c == 'p') { staged_kp = staged_kp.load() + 1.0; }
            else if (c == 'o') { staged_kp = staged_kp.load() - 1.0; if(staged_kp < 0) staged_kp=0; }
            else if (c == 'k') { staged_kd = staged_kd.load() + 0.1; }
            else if (c == 'j') { staged_kd = staged_kd.load() - 0.1; if(staged_kd < 0) staged_kd=0; }
            else if (c == 't') { staged_torque = staged_torque.load() + 0.1; }
            else if (c == 'g') { staged_torque = staged_torque.load() - 0.1; }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

class SingleMotorNode : public rclcpp::Node {
public:
    SingleMotorNode() : Node("single_motor_node") {
        this->declare_parameter("target_motor", "");
        motor_name = this->get_parameter("target_motor").as_string();

        if (motor_name.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Must provide target_motor parameter (e.g. right_hip_pitch_04)");
            exit(1);
        }

        std::string yaml_path;
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("biped_bringup");
            yaml_path = pkg_share + "/config/robot.yaml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to find biped_bringup package: %s", e.what());
            exit(1);
        }

        auto cfg_opt = parse_target_motor(yaml_path, motor_name);
        if (!cfg_opt) {
            RCLCPP_FATAL(this->get_logger(), "Motor %s not found in %s", motor_name.c_str(), yaml_path.c_str());
            exit(1);
        }

        auto cfg = cfg_opt.value();
        motor_interface = cfg.interface;
        motor_id = cfg.id;
        motor_model = cfg.model;

        // Setup raw mapping
        std::unordered_map<std::string, Motor> motors;
        motors[motor_name] = Motor{cfg.id, cfg.model};

        // Zero calibration to allow raw access
        std::unordered_map<std::string, CalibrationEntry> cals;
        cals[motor_name] = CalibrationEntry{1, 0.0};

        try {
            bus_ = std::make_shared<RobstrideBus>(cfg.interface, motors, cals);
            bus_->connect();
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to init or connect RobstrideBus on %s: %s", cfg.interface.c_str(), e.what());
            exit(1);
        }

        pub_ = this->create_publisher<biped_msgs::msg::MotorState>("/" + motor_name + "/motor_state", rclcpp::QoS(1).best_effort());
        cmd_pub_ = this->create_publisher<biped_msgs::msg::MITCommand>("/" + motor_name + "/motor_command", rclcpp::QoS(1).best_effort());

        // Run control loop at 200 Hz
        timer_ = this->create_wall_timer(5ms, std::bind(&SingleMotorNode::loop, this));
        
        input_th_ = std::thread(input_thread);
    }

    ~SingleMotorNode() {
        running = false;
        if (input_th_.joinable()) input_th_.join();
        if (bus_) {
            bus_->disable(motor_name, true);
        }
        std::cout << "\n\nExiting single_motor_node cleanly.\n";
    }

private:
    void loop() {
        if (!running) {
            rclcpp::shutdown();
            return;
        }

        bool is_en = enabled.load();

        if (is_en && !was_enabled_) {
            bus_->enable(motor_name);
            was_enabled_ = true;
        } else if (!is_en && was_enabled_) {
            bus_->disable(motor_name, false);
            was_enabled_ = false;
        }

        if (request_clear_fault.load()) {
            bus_->disable(motor_name, true);
            enabled = false;
            was_enabled_ = false;
            request_clear_fault = false;
        }

        if (is_en) {
            double p = active_pos.load();
            double kp = active_kp.load();
            double kd = active_kd.load();
            double t = active_torque.load();
            bus_->write_operation_frame(motor_name, p, kp, kd, 0.0, t);

            biped_msgs::msg::MITCommand cmd_msg;
            cmd_msg.joint_name = motor_name;
            cmd_msg.position = p;
            cmd_msg.velocity = 0.0;
            cmd_msg.kp = kp;
            cmd_msg.kd = kd;
            cmd_msg.torque_ff = t;
            cmd_pub_->publish(cmd_msg);
        }

        auto fb_opt = bus_->receive_feedback(motor_name, 0.001);
        if (fb_opt) {
            auto fb = fb_opt.value();
            fb_pos = fb.position;
            fb_vel = fb.velocity;
            fb_tau = fb.torque;
            fb_temp = fb.temperature;

            biped_msgs::msg::MotorState msg;
            msg.joint_name = motor_name;
            msg.can_id = motor_id;
            msg.position = fb.position;
            msg.velocity = fb.velocity;
            msg.torque = fb.torque;
            msg.temperature = fb.temperature;
            msg.fault_code = fb.fault_code;
            msg.mode_status = fb.mode_status;
            pub_->publish(msg);
        }
    }

    std::shared_ptr<RobstrideBus> bus_;
    rclcpp::Publisher<biped_msgs::msg::MotorState>::SharedPtr pub_;
    rclcpp::Publisher<biped_msgs::msg::MITCommand>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_th_;
    bool was_enabled_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleMotorNode>();
    
    // Use a custom executor loop to allow checking the running flag
    rclcpp::WallRate rate(200);
    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    running = false; // ensure background thread knows we're stopping
    rclcpp::shutdown();
    return 0;
}
