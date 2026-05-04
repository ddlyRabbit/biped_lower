cmake_minimum_required(VERSION 3.14)
project(biped_driver_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(biped_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Library: biped_driver_cpp_lib ─────────────────────────────────
add_library(biped_driver_cpp_lib STATIC
  src/robstride_bus.cpp
  src/motor_manager.cpp
  src/bno085_reader.cpp
  src/im10a_reader.cpp
)

target_include_directories(biped_driver_cpp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(biped_driver_cpp_lib
  yaml-cpp
)

ament_target_dependencies(biped_driver_cpp_lib
  rclcpp
)

# ── Executable: can_bus_node_cpp ──────────────────────────────────
add_executable(can_bus_node_cpp
  src/can_bus_node.cpp
)

target_link_libraries(can_bus_node_cpp
  biped_driver_cpp_lib
  yaml-cpp
  pthread
)

ament_target_dependencies(can_bus_node_cpp
  rclcpp
  sensor_msgs
  std_msgs
  biped_msgs
)

# ── Executable: single_motor_node_cpp ─────────────────────────────
add_executable(single_motor_node_cpp
  src/single_motor_node.cpp
)

target_link_libraries(single_motor_node_cpp
  biped_driver_cpp_lib
  yaml-cpp
  pthread
)

ament_target_dependencies(single_motor_node_cpp
  rclcpp
  sensor_msgs
  std_msgs
  biped_msgs
  ament_index_cpp
)

# ── Executable: imu_node ──────────────────────────────────────────
add_executable(imu_node
  src/bno085_node.cpp
)

target_link_libraries(imu_node
  pthread
)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# ── Executable: im10a_node ──────────────────────────────────────────
add_executable(im10a_node
  src/im10a_node.cpp
)

target_link_libraries(im10a_node
  biped_driver_cpp_lib
  pthread
)

ament_target_dependencies(im10a_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS can_bus_node_cpp single_motor_node_cpp imu_node im10a_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
