#include "biped_driver_cpp/im10a_reader.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cmath>

namespace biped_driver_cpp {

#define PI 3.14159265358979323846

Im10aReader::Im10aReader() : fd_(-1), initialized_(false), state_(0), current_type_(0), checksum_(0), payload_idx_(0) {
    quat_[0] = 0.0; quat_[1] = 0.0; quat_[2] = 0.0; quat_[3] = 1.0;
    gyro_[0] = 0.0; gyro_[1] = 0.0; gyro_[2] = 0.0;
    accel_[0] = 0.0; accel_[1] = 0.0; accel_[2] = 0.0;
    gravity_[0] = 0.0; gravity_[1] = 0.0; gravity_[2] = 9.81;
}

Im10aReader::~Im10aReader() {
    if (fd_ != -1) {
        close(fd_);
    }
}

bool Im10aReader::configure_serial(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN]  = 0; // read doesn't block
    tty.c_cc[VTIME] = 0; // 0 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }
    return true;
}

bool Im10aReader::auto_baud_and_upgrade() {
    // Basic implementation: try target baud directly first. If no data, try 9600 and upgrade.
    if (!configure_serial(fd_, target_baud_)) return false;
    
    // Give it a moment and check for data
    usleep(100000); 
    uint8_t dummy;
    int n = ::read(fd_, &dummy, 1);
    if (n > 0) {
        // Already at target baud
        return true;
    }

    // Try 9600
    if (!configure_serial(fd_, 9600)) return false;
    usleep(100000);
    n = ::read(fd_, &dummy, 1);
    if (n > 0) {
        // We are at 9600. Send unlock and upgrade commands.
        uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        ::write(fd_, unlock, sizeof(unlock));
        usleep(100000);

        uint8_t baud_code = 0x08; // 460800
        if (target_baud_ == 921600) baud_code = 0x09;
        
        uint8_t set_baud[] = {0xFF, 0xAA, 0x04, baud_code, 0x00};
        ::write(fd_, set_baud, sizeof(set_baud));
        usleep(100000);

        // Switch host to target baud BEFORE sending save command
        if (!configure_serial(fd_, target_baud_)) return false;
        usleep(100000);

        uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
        ::write(fd_, save, sizeof(save));
        usleep(100000);

        return true;
    }

    return false; // Could not detect IMU
}

bool Im10aReader::init(const std::string& port, int target_baud) {
    port_ = port;
    target_baud_ = target_baud;

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Im10aReader: Error opening " << port_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    if (!auto_baud_and_upgrade()) {
        std::cerr << "Im10aReader: Auto-baud / upgrade failed." << std::endl;
        // Proceed anyway, might just be silent for a moment
    }

    // Flush any pending data
    tcflush(fd_, TCIOFLUSH);

    initialized_ = true;
    return true;
}

void Im10aReader::compute_gravity() {
    // Gravity vector from quaternion (assuming upright is Z=-1 in sensor frame)
    // R(q) * [0, 0, -g]^T
    // Based on Python im10a_node.py:
    // gx = 2 * (q[0]*q[2] - q[3]*q[1]) * 9.81
    // gy = 2 * (q[1]*q[2] + q[3]*q[0]) * 9.81
    // gz = (q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]) * 9.81
    // But Isaac Sim expects gravity to be what the sensor *measures* (upward force),
    // and the python node does:
    // q_raw = [euler_quat]
    // gravity_msg.vector.x = 2 * (q_raw[0] * q_raw[2] - q_raw[3] * q_raw[1])
    // gravity_msg.vector.y = 2 * (q_raw[1] * q_raw[2] + q_raw[3] * q_raw[0])
    // gravity_msg.vector.z = (q_raw[3]**2 - q_raw[0]**2 - q_raw[1]**2 + q_raw[2]**2)

    double qx = quat_[0];
    double qy = quat_[1];
    double qz = quat_[2];
    double qw = quat_[3];

    // Compute gravity vector pointing DOWN in world frame, expressed in body frame:
    // R^T * [0, 0, -1] = [ 2(qx*qz - qw*qy), 2(qy*qz + qw*qx), 1 - 2(qx^2 + qy^2) ] * -1
    // Which is: [ -2(qx*qz - qw*qy), -2(qy*qz + qw*qx), 2(qx^2 + qy^2) - 1 ]
    // However, obs_builder.cpp expects the sensor to report UPWARD acceleration (like BNO085),
    // which is (0, 0, +9.81) when upright. So we compute R^T * [0, 0, 1] instead:
    
    gravity_[0] = 2.0 * (qx * qz - qw * qy);
    gravity_[1] = 2.0 * (qy * qz + qw * qx);
    gravity_[2] = (qw * qw - qx * qx - qy * qy + qz * qz);

    // Normalize and scale to +9.81
    double norm = std::sqrt(gravity_[0]*gravity_[0] + gravity_[1]*gravity_[1] + gravity_[2]*gravity_[2]);
    if (norm > 0.001) {
        gravity_[0] = (gravity_[0] / norm) * 9.81;
        gravity_[1] = (gravity_[1] / norm) * 9.81;
        gravity_[2] = (gravity_[2] / norm) * 9.81;
    }
}

void Im10aReader::process_packet(uint8_t type, const uint8_t* data) {
    if (type == 0x51) { // Accel
        int16_t ax = (int16_t)((data[1] << 8) | data[0]);
        int16_t ay = (int16_t)((data[3] << 8) | data[2]);
        int16_t az = (int16_t)((data[5] << 8) | data[4]);
        accel_[0] = (ax / 32768.0) * 16.0 * 9.81;
        accel_[1] = (ay / 32768.0) * 16.0 * 9.81;
        accel_[2] = (az / 32768.0) * 16.0 * 9.81;
    } 
    else if (type == 0x52) { // Gyro
        int16_t wx = (int16_t)((data[1] << 8) | data[0]);
        int16_t wy = (int16_t)((data[3] << 8) | data[2]);
        int16_t wz = (int16_t)((data[5] << 8) | data[4]);
        gyro_[0] = (wx / 32768.0) * 2000.0 * (PI / 180.0);
        gyro_[1] = (wy / 32768.0) * 2000.0 * (PI / 180.0);
        gyro_[2] = (wz / 32768.0) * 2000.0 * (PI / 180.0);
    }
    else if (type == 0x59) { // Quaternion
        int16_t q0 = (int16_t)((data[1] << 8) | data[0]);
        int16_t q1 = (int16_t)((data[3] << 8) | data[2]);
        int16_t q2 = (int16_t)((data[5] << 8) | data[4]);
        int16_t q3 = (int16_t)((data[7] << 8) | data[6]);
        quat_[0] = q1 / 32768.0; // x
        quat_[1] = q2 / 32768.0; // y
        quat_[2] = q3 / 32768.0; // z
        quat_[3] = q0 / 32768.0; // w
        
        compute_gravity();
    }
}

ImuData Im10aReader::read() {
    ImuData output;
    if (!initialized_ || fd_ < 0) return output;

    uint8_t buf[256];
    int n = ::read(fd_, buf, sizeof(buf));
    
    if (n > 0) {
        for (int i = 0; i < n; ++i) {
            uint8_t b = buf[i];
            switch (state_) {
                case 0: // Wait for 0x55
                    if (b == 0x55) {
                        state_ = 1;
                        checksum_ = 0x55;
                    }
                    break;
                case 1: // Get type
                    if (b >= 0x50 && b <= 0x5A) {
                        current_type_ = b;
                        checksum_ += b;
                        payload_idx_ = 0;
                        state_ = 2;
                    } else if (b == 0x55) {
                        // Might be back-to-back 0x55
                        state_ = 1;
                        checksum_ = 0x55;
                    } else {
                        state_ = 0;
                    }
                    break;
                case 2: // Get payload (8 bytes)
                    payload_[payload_idx_++] = b;
                    checksum_ += b;
                    if (payload_idx_ == 8) {
                        state_ = 3;
                    }
                    break;
                case 3: // Checksum
                    if (b == checksum_) {
                        process_packet(current_type_, payload_);
                    }
                    state_ = 0;
                    break;
            }
        }
    }

    output.quaternion[0] = quat_[0];
    output.quaternion[1] = quat_[1];
    output.quaternion[2] = quat_[2];
    output.quaternion[3] = quat_[3];
    
    output.gyro[0] = gyro_[0];
    output.gyro[1] = gyro_[1];
    output.gyro[2] = gyro_[2];

    output.accel[0] = accel_[0];
    output.accel[1] = accel_[1];
    output.accel[2] = accel_[2];

    output.gravity[0] = gravity_[0];
    output.gravity[1] = gravity_[1];
    output.gravity[2] = gravity_[2];

    output.valid = initialized_;
    return output;
}

} // namespace biped_driver_cpp