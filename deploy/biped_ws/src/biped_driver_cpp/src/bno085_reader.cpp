/**
 * @file bno085_reader.cpp
 * @brief Bare-metal BNO085 IMU reader implementation.
 *
 * Extracted from bno085_node.cpp — identical I2C protocol,
 * but with no ROS dependencies. Used by the unified node.
 */

#include "biped_driver_cpp/bno085_reader.hpp"

#include <cstring>
#include <stdexcept>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>

namespace biped_driver_cpp {

Bno085Reader::~Bno085Reader() {
    if (fd_ >= 0) close(fd_);
}

bool Bno085Reader::init(int i2c_bus, int i2c_address, int reset_pin,
                         double rate_hz, bool use_game_quat) {
    use_game_quat_ = use_game_quat;

    // Hardware reset via GPIO (use python lgpio — same as original node)
    if (reset_pin >= 0) {
        std::string cmd = "python3 -c \"import lgpio, time; "
                          "h=lgpio.gpiochip_open(4); "
                          "lgpio.gpio_claim_output(h, " + std::to_string(reset_pin) + ", 0); "
                          "time.sleep(0.1); "
                          "lgpio.gpio_write(h, " + std::to_string(reset_pin) + ", 1); "
                          "time.sleep(1.0); "
                          "lgpio.gpio_free(h, " + std::to_string(reset_pin) + "); "
                          "lgpio.gpiochip_close(h)\" 2>/dev/null";
        (void)system(cmd.c_str());
    }

    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", i2c_bus);
    fd_ = open(filename, O_RDWR);
    if (fd_ < 0) return false;

    if (ioctl(fd_, I2C_SLAVE, i2c_address) < 0) {
        close(fd_);
        fd_ = -1;
        return false;
    }

    uint32_t interval_us = static_cast<uint32_t>(1000000.0 / rate_hz);

    for (int attempt = 0; attempt < 3; ++attempt) {
        try {
            if (use_game_quat_) {
                enable_feature(0x08, interval_us);
            } else {
                enable_feature(0x05, interval_us);
            }
            enable_feature(0x02, interval_us);  // Gyroscope
            enable_feature(0x06, interval_us);  // Gravity
            return true;
        } catch (const std::exception&) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    close(fd_);
    fd_ = -1;
    return false;
}

void Bno085Reader::enable_feature(uint8_t report_id, uint32_t interval_us) {
    uint8_t buffer[21] = {};
    buffer[0] = 21;  // Length LSB
    buffer[1] = 0;   // Length MSB
    buffer[2] = 2;   // Channel (Control)
    buffer[3] = 0;   // Sequence number
    buffer[4] = 0xFD; // Set Feature Command
    buffer[5] = report_id;
    buffer[9] = (interval_us & 0xFF);
    buffer[10] = ((interval_us >> 8) & 0xFF);
    buffer[11] = ((interval_us >> 16) & 0xFF);
    buffer[12] = ((interval_us >> 24) & 0xFF);

    if (write(fd_, buffer, 21) != 21) {
        throw std::runtime_error("I2C write failed for Set Feature");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

ImuData Bno085Reader::read() {
    ImuData result;
    if (fd_ < 0) return result;

    try {
        // Drain FIFO — process up to 10 packets
        for (int i = 0; i < 10; ++i) {
            uint8_t header[4];
            if (::read(fd_, header, 4) != 4) break;

            uint16_t packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;
            if (packet_len == 0 || packet_len > 512) break;

            uint8_t buf[512];
            if (::read(fd_, buf, packet_len) != packet_len) break;

            uint8_t channel = buf[2];
            if (channel != 3) continue;  // Not Sensor Report

            int offset = 4;
            while (offset < packet_len) {
                uint8_t report_id = buf[offset];
                if (report_id == 0xFB) {  // Base Timestamp
                    offset += 5;
                } else if (report_id == 0x05 || report_id == 0x08) {
                    // Rotation Vector / Game Rotation Vector
                    int len = (report_id == 0x05) ? 14 : 12;
                    if (offset + len > packet_len) break;
                    auto decode_q = [&](int base) {
                        int16_t ix   = (buf[base+1] << 8) | buf[base];
                        int16_t jx   = (buf[base+3] << 8) | buf[base+2];
                        int16_t kx   = (buf[base+5] << 8) | buf[base+4];
                        int16_t real = (buf[base+7] << 8) | buf[base+6];
                        quat_[0] = ix / 16384.0;
                        quat_[1] = jx / 16384.0;
                        quat_[2] = kx / 16384.0;
                        quat_[3] = real / 16384.0;
                    };
                    decode_q(offset + 4);
                    offset += len;
                } else if (report_id == 0x02) {
                    // Gyroscope
                    if (offset + 10 > packet_len) break;
                    auto decode_g = [&](int base) {
                        int16_t x = (buf[base+1] << 8) | buf[base];
                        int16_t y = (buf[base+3] << 8) | buf[base+2];
                        int16_t z = (buf[base+5] << 8) | buf[base+4];
                        gyro_[0] = x / 512.0;
                        gyro_[1] = y / 512.0;
                        gyro_[2] = z / 512.0;
                    };
                    decode_g(offset + 4);
                    offset += 10;
                } else if (report_id == 0x06) {
                    // Gravity
                    if (offset + 10 > packet_len) break;
                    auto decode_gr = [&](int base) {
                        int16_t x = (buf[base+1] << 8) | buf[base];
                        int16_t y = (buf[base+3] << 8) | buf[base+2];
                        int16_t z = (buf[base+5] << 8) | buf[base+4];
                        
                        double gx = -x / 256.0;
                        double gy = -y / 256.0;
                        double gz = -z / 256.0;
                        
                        // Normalize to unit vector to match Isaac convention
                        double norm = std::sqrt(gx*gx + gy*gy + gz*gz);
                        if (norm > 0.1) {
                            gravity_[0] = gx / norm;
                            gravity_[1] = gy / norm;
                            gravity_[2] = gz / norm;
                        } else {
                            gravity_[0] = 0.0;
                            gravity_[1] = 0.0;
                            gravity_[2] = -1.0;
                        }
                    };
                    decode_gr(offset + 4);
                    offset += 10;
                } else {
                    break;  // Unknown report
                }
            }
        }
        read_count_++;
        result.valid = true;
    } catch (...) {
        error_count_++;
    }

    std::memcpy(result.quaternion, quat_, sizeof(quat_));
    std::memcpy(result.gyro, gyro_, sizeof(gyro_));
    std::memcpy(result.gravity, gravity_, sizeof(gravity_));
    return result;
}

}  // namespace biped_driver_cpp
