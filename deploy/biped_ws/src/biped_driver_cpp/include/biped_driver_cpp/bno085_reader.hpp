/**
 * @file bno085_reader.hpp
 * @brief Bare-metal BNO085 IMU reader (I2C, no ROS dependency).
 *
 * Extracted from bno085_node.cpp for use in the unified sense-and-control node.
 * Drains the I2C FIFO on each read(), returns the latest quaternion, gyro, and gravity.
 */

#pragma once

#include <cstdint>
#include <string>
#include <array>

namespace biped_driver_cpp {

struct ImuData {
    double quaternion[4] = {0.0, 0.0, 0.0, 1.0};  // x, y, z, w
    double gyro[3]       = {0.0, 0.0, 0.0};         // rad/s
    double accel[3]      = {0.0, 0.0, 0.0};         // m/s^2
    double gravity[3]    = {0.0, 0.0, -1.0};         // Normalized unit vector
    bool valid = false;
};

class Bno085Reader {
public:
    Bno085Reader() = default;
    ~Bno085Reader();

    /// Initialize I2C bus. Returns true on success.
    bool init(int i2c_bus, int i2c_address, int reset_pin = -1,
              double rate_hz = 200.0, bool use_game_quat = false);

    /// Drain FIFO and return latest data.
    ImuData read();

    bool is_initialized() const { return fd_ >= 0; }

private:
    void enable_feature(uint8_t report_id, uint32_t interval_us);

    int fd_ = -1;
    bool use_game_quat_ = false;

    // Latest state (updated by read)
    double quat_[4]   = {0.0, 0.0, 0.0, 1.0};
    double gyro_[3]   = {0.0, 0.0, 0.0};
    double gravity_[3] = {0.0, 0.0, -1.0};

    uint64_t read_count_  = 0;
    uint64_t error_count_ = 0;
};

}  // namespace biped_driver_cpp
