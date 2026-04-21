import os

filepath = 'deploy/biped_ws/src/biped_driver_cpp/src/can_bus_node.cpp'
with open(filepath, 'r') as f:
    content = f.read()

content = content.replace(
    '        while (running_) {\n            // Wait for command or 25ms timeout — avoids busy spinning',
    '        auto last_t0 = std::chrono::steady_clock::now();\n\n        while (running_) {\n            // Wait for command or 25ms timeout — avoids busy spinning'
)

content = content.replace(
    '            auto dt = std::chrono::duration<double>(\n                std::chrono::steady_clock::now() - t0).count();\n            buffer_->update_stats(dt);',
    '            auto dt = std::chrono::duration<double>(t0 - last_t0).count();\n            last_t0 = t0;\n            buffer_->update_stats(dt);'
)

with open(filepath, 'w') as f:
    f.write(content)

print('Stats fixed.')
