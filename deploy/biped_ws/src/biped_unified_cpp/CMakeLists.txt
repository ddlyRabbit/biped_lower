cmake_minimum_required(VERSION 3.14)
project(biped_unified_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(biped_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# onnxruntime
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_library(ONNXRUNTIME_LIB onnxruntime)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    PATH_SUFFIXES onnxruntime/core/session onnxruntime)
endif()

# ── Executable: unified_node ─────────────────────────────────────
add_executable(unified_node src/unified_node.cpp)

target_link_libraries(unified_node
  biped_driver_cpp_lib
  biped_control_cpp_lib
  yaml-cpp
  pthread
)

if(onnxruntime_FOUND)
  target_link_libraries(unified_node onnxruntime::onnxruntime)
else()
  if(ONNXRUNTIME_LIB)
    target_link_libraries(unified_node ${ONNXRUNTIME_LIB})
  endif()
  if(ONNXRUNTIME_INCLUDE_DIR)
    target_include_directories(unified_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  endif()
endif()

ament_target_dependencies(unified_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  biped_msgs
)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS unified_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
