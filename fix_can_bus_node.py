import os

filepath = 'deploy/biped_ws/src/biped_driver_cpp/src/can_bus_node.cpp'
with open(filepath, 'r') as f:
    content = f.read()

# 1. Add the boolean flag to SharedBuffer private members
content = content.replace(
    '    std::unordered_map<std::string, FeedbackEntry> feedback_;\n    double loop_dt_ = 0.0;\n    uint64_t loop_count_ = 0;\n};',
    '    std::unordered_map<std::string, FeedbackEntry> feedback_;\n    double loop_dt_ = 0.0;\n    uint64_t loop_count_ = 0;\n    bool new_command_ready_ = false;\n};'
)

# 2. Update notify_new_commands() to set the flag
content = content.replace(
    '    void notify_new_commands() {\n        cmd_cv_.notify_all();\n    }',
    '    void notify_new_commands() {\n        {\n            std::lock_guard<std::mutex> lk(cmd_cv_mtx_);\n            new_command_ready_ = true;\n        }\n        cmd_cv_.notify_all();\n    }'
)

# 3. Update wait_for_command() to use the predicate and reset the flag
old_wait = '''    bool wait_for_command(int timeout_ms = 25) {
        std::unique_lock<std::mutex> lk(cmd_cv_mtx_);
        return cmd_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms))
               == std::cv_status::no_timeout;
    }'''

new_wait = '''    bool wait_for_command(int timeout_ms = 25) {
        std::unique_lock<std::mutex> lk(cmd_cv_mtx_);
        bool woken_by_cmd = cmd_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), 
                                             [this] { return new_command_ready_; });
        new_command_ready_ = false;
        return woken_by_cmd;
    }'''

content = content.replace(old_wait, new_wait)

with open(filepath, 'w') as f:
    f.write(content)
print('can_bus_node.cpp patched.')
