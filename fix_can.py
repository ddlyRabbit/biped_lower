import re

with open("/home/abhinavroy/biped_lower/deploy/biped_ws/src/biped_driver_cpp/src/can_bus_node.cpp", "r") as f:
    content = f.read()

# Replace the broken cmds collection logic
bad_code = """            // Get commands
            auto cmds = mgr_->commands();
            for (const auto& [name, cmd] : cmds) {
                row.cmd_pos[name] = cmd.position;
            }"""

good_code = """            // Get commands
            for (auto& [bus_name, buf] : buffers_) {
                auto cmds = buf->read_commands();
                for (const auto& [name, cmd] : cmds) {
                    row.cmd_pos[name] = cmd.position;
                }
            }"""

content = content.replace(bad_code, good_code)

with open("/home/abhinavroy/biped_lower/deploy/biped_ws/src/biped_driver_cpp/src/can_bus_node.cpp", "w") as f:
    f.write(content)
