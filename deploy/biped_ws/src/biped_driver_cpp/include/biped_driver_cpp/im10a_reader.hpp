#ifndef BIPED_DRIVER_CPP_IM10A_READER_HPP_
#define BIPED_DRIVER_CPP_IM10A_READER_HPP_

#include <string>
#include <vector>
#include <stdint.h>
#include "biped_driver_cpp/bno085_reader.hpp" // For ImuData struct
#include "biped_driver_cpp/imu_filter.hpp"

namespace biped_driver_cpp {

class Im10aReader {
public:
    Im10aReader();
    ~Im10aReader();

    bool init(const std::string& port = "/dev/ttyUSB0", int target_baud = 460800);
    ImuData read();

    /// Enable Butterworth low-pass on gyro/gravity, applied per parsed frame
    /// (~sample_hz). Call after init(); cutoff <= 0 or >= 0.45*sample_hz = off.
    void set_filter(double gyro_cutoff_hz, double gravity_cutoff_hz, double sample_hz) {
        gyro_filter_.init(gyro_cutoff_hz, sample_hz);
        gravity_filter_.init(gravity_cutoff_hz, sample_hz);
    }

private:
    int fd_;
    bool initialized_;
    bool has_new_data_;
    std::string port_;
    int target_baud_;

    // State machine for parsing
    std::vector<uint8_t> buffer_;
    int state_;
    uint8_t current_type_;
    uint8_t payload_[8];
    uint8_t checksum_;
    int payload_idx_;

    // Latest data
    double quat_[4]; // x, y, z, w
    double gyro_[3];
    double accel_[3];
    double gravity_[3];

    // Optional low-pass filters (bypass by default)
    Vec3Filter gyro_filter_;
    Vec3Filter gravity_filter_;

    bool configure_serial(int fd, int baud);
    bool auto_baud_and_upgrade();
    void process_packet(uint8_t type, const uint8_t* data);
    void compute_gravity();
};

} // namespace biped_driver_cpp

#endif // BIPED_DRIVER_CPP_IM10A_READER_HPP_