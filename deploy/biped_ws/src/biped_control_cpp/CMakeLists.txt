cmake_minimum_required(VERSION 3.14)
project(biped_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(biped_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Try to find ONNXRuntime (assuming system installed or in CMAKE_PREFIX_PATH)
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  message(WARNING "onnxruntime package not found, linking manually...")
  find_library(ONNXRUNTIME_LIB onnxruntime)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h PATH_SUFFIXES onnxruntime/core/session onnxruntime)
endif()

# ── Library: biped_control_cpp_lib ─────────────────────────────────
add_library(biped_control_cpp_lib STATIC
  src/obs_builder.cpp
)

target_include_directories(biped_control_cpp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(biped_control_cpp_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  biped_msgs
)

# ── Executable: state_machine_node_cpp ────────────────────────────
add_executable(state_machine_node_cpp src/state_machine_node.cpp)
target_link_libraries(state_machine_node_cpp biped_control_cpp_lib yaml-cpp pthread)
ament_target_dependencies(state_machine_node_cpp
  rclcpp sensor_msgs std_msgs geometry_msgs biped_msgs
)

# ── Executable: safety_node_cpp ───────────────────────────────────
add_executable(safety_node_cpp src/safety_node.cpp)
target_link_libraries(safety_node_cpp biped_control_cpp_lib pthread)
ament_target_dependencies(safety_node_cpp
  rclcpp sensor_msgs std_msgs geometry_msgs biped_msgs
)

# ── Executable: policy_node_cpp ───────────────────────────────────
add_executable(policy_node_cpp src/policy_node.cpp)
target_link_libraries(policy_node_cpp biped_control_cpp_lib yaml-cpp pthread)

if(onnxruntime_FOUND)
  target_link_libraries(policy_node_cpp onnxruntime::onnxruntime)
else()
  if(ONNXRUNTIME_LIB)
    target_link_libraries(policy_node_cpp ${ONNXRUNTIME_LIB})
  endif()
  if(ONNXRUNTIME_INCLUDE_DIR)
    target_include_directories(policy_node_cpp PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  endif()
endif()

ament_target_dependencies(policy_node_cpp
  rclcpp sensor_msgs std_msgs geometry_msgs biped_msgs
)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS state_machine_node_cpp safety_node_cpp policy_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()