import re

filepath = "/home/abhinavroy/biped_lower/deploy/biped_ws/src/biped_driver_cpp/src/can_bus_node.cpp"
with open(filepath, "r") as f:
    content = f.read()

# 1. Add headers
if "<fstream>" not in content:
    content = content.replace("#include <yaml-cpp/yaml.h>", "#include <yaml-cpp/yaml.h>\n#include <fstream>\n#include <std_msgs/msg/string.hpp>")

# 2. Add member variables for recording
members_str = """
    double pub_rate_;
    bool is_chirping_ = false;
    
    struct RecordRow {
        double time;
        std::map<std::string, double> cmd_pos;
        std::map<std::string, double> pos;
        std::map<std::string, double> vel;
        std::map<std::string, double> tau;
    };
    std::vector<RecordRow> recording_buffer_;
    rclcpp::Subscription<std::string>::SharedPtr state_sub_;
"""
content = re.sub(r'    double pub_rate_;', members_str, content)

# 3. Add state subscriber to constructor
sub_code = """
        // ── Recording Subscriber ─────────────────────────────────
        state_sub_ = create_subscription<std::string>(
            "/state_machine/state", 10,
            [this](const std::string::SharedPtr msg) {
                bool new_state = (msg->data == "CHIRP");
                if (new_state && !is_chirping_) {
                    RCLCPP_INFO(get_logger(), "Started CHIRP recording at %.1f Hz", pub_rate_);
                    recording_buffer_.clear();
                    is_chirping_ = true;
                } else if (!new_state && is_chirping_) {
                    RCLCPP_INFO(get_logger(), "Stopped CHIRP recording. Saving %zu samples...", recording_buffer_.size());
                    is_chirping_ = false;
                    save_recording();
                }
            }
        );
"""
content = content.replace("// ── Start Workers ────────────────────────────────────────", sub_code + "\n        // ── Start Workers ────────────────────────────────────────")

# 4. Add save_recording method
save_method = """
    void save_recording() {
        if (recording_buffer_.empty()) return;
        
        std::ofstream file("sysid_data_400hz.csv");
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open sysid_data_400hz.csv for writing!");
            return;
        }
        
        // Write header
        file << "time";
        std::vector<std::string> joint_names;
        for (const auto& [name, _] : recording_buffer_[0].pos) {
            joint_names.push_back(name);
            file << "," << name << "_target," << name << "_pos," << name << "_vel," << name << "_tau";
        }
        file << "\\n";
        
        // Write data
        for (const auto& row : recording_buffer_) {
            file << std::fixed << std::setprecision(6) << row.time;
            for (const auto& name : joint_names) {
                file << "," << (row.cmd_pos.count(name) ? row.cmd_pos.at(name) : 0.0)
                     << "," << (row.pos.count(name) ? row.pos.at(name) : 0.0)
                     << "," << (row.vel.count(name) ? row.vel.at(name) : 0.0)
                     << "," << (row.tau.count(name) ? row.tau.at(name) : 0.0);
            }
            file << "\\n";
        }
        file.close();
        RCLCPP_INFO(get_logger(), "Saved sysid_data_400hz.csv successfully.");
        recording_buffer_.clear();
    }
"""
content = content.replace("void build_motor_groups() {", save_method + "\n    void build_motor_groups() {")

# 5. Inject recording logic into publish_loop
record_logic = """
        // ── Record Data if in CHIRP mode ─────────────────────────
        if (is_chirping_) {
            RecordRow row;
            row.time = now.seconds();
            
            // Get commands
            auto cmds = mgr_->commands();
            for (const auto& [name, cmd] : cmds) {
                row.cmd_pos[name] = cmd.position;
            }
            
            // Get feedback from ROS message
            for (size_t i = 0; i < joint_msg.name.size(); ++i) {
                std::string name = joint_msg.name[i];
                row.pos[name] = joint_msg.position[i];
                row.vel[name] = joint_msg.velocity[i];
                row.tau[name] = joint_msg.effort[i];
            }
            recording_buffer_.push_back(row);
        }
"""
content = content.replace("joint_pub_->publish(joint_msg);", record_logic + "\n        joint_pub_->publish(joint_msg);")

# 6. Change default publish_rate parameter to 400.0
content = content.replace('declare_parameter("publish_rate", 200.0);', 'declare_parameter("publish_rate", 400.0);')

with open(filepath, "w") as f:
    f.write(content)
