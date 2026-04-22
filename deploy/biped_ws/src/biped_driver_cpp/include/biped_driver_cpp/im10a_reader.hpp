#ifndef BIPED_DRIVER_CPP_IM10A_READER_HPP_
#define BIPED_DRIVER_CPP_IM10A_READER_HPP_

#include <string>
#include <vector>
#include <stdint.h>
#include "biped_driver_cpp/bno085_reader.hpp" // For ImuData struct

namespace biped_driver_cpp {

class Im10aReader {
public:
    Im10aReader();
    ~Im10aReader();

    bool init(const std::string& port = "/dev/ttyUSB0", int target_baud = 460800);
    ImuData read();

private:
    int fd_;
    bool initialized_;
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

    bool configure_serial(int fd, int baud);
    bool auto_baud_and_upgrade();
    void process_packet(uint8_t type, const uint8_t* data);
    void compute_gravity();
};

} // namespace biped_driver_cpp

#endif // BIPED_DRIVER_CPP_IM10A_READER_HPP_