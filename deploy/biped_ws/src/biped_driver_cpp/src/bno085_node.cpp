#include <chrono>
#include <string>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Bno085NodeCpp : public rclcpp::Node {
public:
    Bno085NodeCpp() : Node("imu_node") {
        // Parameters
        declare_parameter("i2c_bus", 1); // Jetson Orin Nano: bus 7, RPi5: bus 1
        declare_parameter("i2c_address", 0x4B);
        declare_parameter("rate_hz", 200.0);
        declare_parameter("frame_id", "imu_link");
        declare_parameter("use_game_quaternion", false);
        declare_parameter("reset_pin", 7);

        i2c_bus_ = get_parameter("i2c_bus").as_int();
        i2c_addr_ = get_parameter("i2c_address").as_int();
        rate_hz_ = get_parameter("rate_hz").as_double();
        frame_id_ = get_parameter("frame_id").as_string();
        use_game_quat_ = get_parameter("use_game_quaternion").as_bool();
        reset_pin_ = get_parameter("reset_pin").as_int();

        // QoS: best-effort for real-time sensor data
        auto sensor_qos = rclcpp::QoS(1);
        sensor_qos.best_effort();

        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", sensor_qos);
        pub_grav_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/gravity", sensor_qos);
        
        // TF broadcaster: odom -> base_link from IMU orientation
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        init_imu();

        // Timer at configured rate
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            std::bind(&Bno085NodeCpp::timer_callback, this)
        );

        RCLCPP_INFO(get_logger(), "BNO085 IMU started (C++) — I2C bus %d, addr 0x%02X, %.1fHz, game_quat=%d",
                    i2c_bus_, i2c_addr_, rate_hz_, use_game_quat_);
    }

    ~Bno085NodeCpp() {
        if (file_ >= 0) {
            close(file_);
        }
    }

private:
    int i2c_bus_;
    int i2c_addr_;
    double rate_hz_;
    std::string frame_id_;
    bool use_game_quat_;
    int reset_pin_;
    int file_ = -1;

    // State — latest readings
    double last_quat_[4] = {0.0, 0.0, 0.0, 1.0}; // x, y, z, w
    double last_gyro_[3] = {0.0, 0.0, 0.0};      // rad/s
    double last_gravity_[3] = {0.0, 0.0, -1.0}; // m/s^2

    // Diagnostics
    uint64_t read_count_ = 0;
    uint64_t error_count_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_grav_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_imu() {
        if (reset_pin_ >= 0) {
            RCLCPP_INFO(get_logger(), "Resetting BNO085 via Python lgpio on pin %d...", reset_pin_);
            // Use python to execute the lgpio hardware reset logic exactly like python node
            std::string cmd = "python3 -c \"import lgpio, time; "
                              "h=lgpio.gpiochip_open(4); "
                              "lgpio.gpio_claim_output(h, " + std::to_string(reset_pin_) + ", 0); "
                              "time.sleep(0.1); "
                              "lgpio.gpio_write(h, " + std::to_string(reset_pin_) + ", 1); "
                              "time.sleep(1.0); "
                              "lgpio.gpio_free(h, " + std::to_string(reset_pin_) + "); "
                              "lgpio.gpiochip_close(h)\" 2>/dev/null";
            int ret = system(cmd.c_str());
            if (ret != 0) {
                RCLCPP_WARN(get_logger(), "GPIO reset script failed (returned %d), continuing without reset", ret);
            } else {
                RCLCPP_INFO(get_logger(), "BNO085 hardware reset via GPIO %d", reset_pin_);
            }
        }

        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", i2c_bus_);
        file_ = open(filename, O_RDWR);
        if (file_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open I2C bus %d", i2c_bus_);
            return;
        }

        if (ioctl(file_, I2C_SLAVE, i2c_addr_) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to acquire bus access to slave 0x%02X", i2c_addr_);
            return;
        }

        // Enable reports with retry
        uint32_t interval_us = static_cast<uint32_t>(1000000.0 / rate_hz_);
        int max_retries = 3;
        bool success = false;

        for (int attempt = 0; attempt < max_retries; attempt++) {
            try {
                if (use_game_quat_) {
                    enable_feature(0x08, interval_us); // GAME_ROTATION_VECTOR
                    RCLCPP_INFO(get_logger(), "Using GAME_ROTATION_VECTOR (no mag correction)");
                } else {
                    enable_feature(0x05, interval_us); // ROTATION_VECTOR
                    RCLCPP_INFO(get_logger(), "Using ROTATION_VECTOR (mag-corrected)");
                }
                enable_feature(0x02, interval_us); // GYROSCOPE
                enable_feature(0x06, interval_us); // GRAVITY
                
                success = true;
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Feature enable failed (attempt %d): %s", attempt + 1, e.what());
                std::this_thread::sleep_for(500ms);
            }
        }

        if (success) {
            RCLCPP_INFO(get_logger(), "BNO085 initialized — reports enabled at %u us interval", interval_us);
        } else {
            RCLCPP_ERROR(get_logger(), "BNO085 init failed after %d retries", max_retries);
            close(file_);
            file_ = -1;
        }
    }

    void enable_feature(uint8_t report_id, uint32_t interval_us) {
        uint8_t buffer[21] = {0};
        buffer[0] = 21; // Length LSB
        buffer[1] = 0;  // Length MSB
        buffer[2] = 2;  // Channel (Control)
        buffer[3] = 0;  // Sequence number
        
        buffer[4] = 0xFD; // Set Feature Command
        buffer[5] = report_id;
        buffer[6] = 0; // Feature Flags
        buffer[7] = 0; // Change Sensitivity LSB
        buffer[8] = 0; // Change Sensitivity MSB
        
        buffer[9] = (interval_us & 0xFF);
        buffer[10] = ((interval_us >> 8) & 0xFF);
        buffer[11] = ((interval_us >> 16) & 0xFF);
        buffer[12] = ((interval_us >> 24) & 0xFF);
        
        // Batch interval (4 bytes, 0)
        // Sensor-specific (4 bytes, 0)
        
        if (write(file_, buffer, 21) != 21) {
            throw std::runtime_error("I2C write failed for Set Feature");
        }
        std::this_thread::sleep_for(50ms); // Allow sensor to process
    }

    void read_sensor() {
        if (file_ < 0) return;

        try {
            // Drain the I2C FIFO (process up to 10 packets to ensure latest data)
            for (int i = 0; i < 10; i++) {
                uint8_t header[4];
                if (read(file_, header, 4) != 4) break;
                
                uint16_t packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;
                if (packet_len == 0 || packet_len > 512) break; // 0 = no data, > 512 = corrupt length
                
                uint8_t buf[512];
                if (read(file_, buf, packet_len) != packet_len) break;
                
                uint8_t channel = buf[2];
                if (channel == 3) { // Sensor Report channel
                    int offset = 4;
                    while (offset < packet_len) {
                        uint8_t report_id = buf[offset];
                        if (report_id == 0xFB) { // Base Timestamp
                            offset += 5;
                        } else if (report_id == 0x05) { // Rotation Vector
                            if (offset + 14 > packet_len) break;
                            int16_t ix = (buf[offset+5] << 8) | buf[offset+4];
                            int16_t jx = (buf[offset+7] << 8) | buf[offset+6];
                            int16_t kx = (buf[offset+9] << 8) | buf[offset+8];
                            int16_t realx = (buf[offset+11] << 8) | buf[offset+10];
                            last_quat_[0] = ix / 16384.0;
                            last_quat_[1] = jx / 16384.0;
                            last_quat_[2] = kx / 16384.0;
                            last_quat_[3] = realx / 16384.0;
                            offset += 14;
                        } else if (report_id == 0x08) { // Game Rotation Vector
                            if (offset + 12 > packet_len) break;
                            int16_t ix = (buf[offset+5] << 8) | buf[offset+4];
                            int16_t jx = (buf[offset+7] << 8) | buf[offset+6];
                            int16_t kx = (buf[offset+9] << 8) | buf[offset+8];
                            int16_t realx = (buf[offset+11] << 8) | buf[offset+10];
                            last_quat_[0] = ix / 16384.0;
                            last_quat_[1] = jx / 16384.0;
                            last_quat_[2] = kx / 16384.0;
                            last_quat_[3] = realx / 16384.0;
                            offset += 12;
                        } else if (report_id == 0x02) { // Gyroscope
                            if (offset + 10 > packet_len) break;
                            int16_t x = (buf[offset+5] << 8) | buf[offset+4];
                            int16_t y = (buf[offset+7] << 8) | buf[offset+6];
                            int16_t z = (buf[offset+9] << 8) | buf[offset+8];
                            last_gyro_[0] = x / 512.0;
                            last_gyro_[1] = y / 512.0;
                            last_gyro_[2] = z / 512.0;
                            offset += 10;
                        } else if (report_id == 0x06) { // Gravity
                            if (offset + 10 > packet_len) break;
                            int16_t x = (buf[offset+5] << 8) | buf[offset+4];
                            int16_t y = (buf[offset+7] << 8) | buf[offset+6];
                            int16_t z = (buf[offset+9] << 8) | buf[offset+8];
                            double gx = -x / 256.0;
                            double gy = -y / 256.0;
                            double gz = -z / 256.0;
                            
                            // Normalize to unit vector to match Isaac convention
                            double norm = std::sqrt(gx*gx + gy*gy + gz*gz);
                            if (norm > 0.1) {
                                last_gravity_[0] = gx / norm;
                                last_gravity_[1] = gy / norm;
                                last_gravity_[2] = gz / norm;
                            } else {
                                last_gravity_[0] = 0.0;
                                last_gravity_[1] = 0.0;
                                last_gravity_[2] = -1.0;
                            }
                            offset += 10;
                        } else {
                            // Unknown report ID or unhandled data format, abort parsing to avoid endless loop
                            break; 
                        }
                    }
                }
            }
            read_count_++;
        } catch (const std::exception& e) {
            error_count_++;
            if (error_count_ % 100 == 1) {
                RCLCPP_WARN(get_logger(), "IMU read error (%lu total): %s", error_count_, e.what());
            }
        }
    }

    void timer_callback() {
        // Read sensor up to 10 packets to clear FIFO
        read_sensor();
        
        auto now = this->now();

        // --- Publish /imu/data ---
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = frame_id_;

        // Orientation (quaternion, ROS convention: x, y, z, w)
        imu_msg.orientation.x = last_quat_[0];
        imu_msg.orientation.y = last_quat_[1];
        imu_msg.orientation.z = last_quat_[2];
        imu_msg.orientation.w = last_quat_[3];

        // Angular velocity (rad/s, body frame)
        imu_msg.angular_velocity.x = last_gyro_[0];
        imu_msg.angular_velocity.y = last_gyro_[1];
        imu_msg.angular_velocity.z = last_gyro_[2];

        // Linear acceleration (not populated, use gravity topic)
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
        // Covariance: -1 in first element = unknown
        imu_msg.linear_acceleration_covariance[0] = -1.0;

        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.angular_velocity_covariance[0] = -1.0;

        pub_imu_->publish(imu_msg);

        // --- Publish /imu/gravity ---
        auto grav_msg = geometry_msgs::msg::Vector3Stamped();
        grav_msg.header.stamp = now;
        grav_msg.header.frame_id = frame_id_;
        grav_msg.vector.x = last_gravity_[0];
        grav_msg.vector.y = last_gravity_[1];
        grav_msg.vector.z = last_gravity_[2];

        pub_grav_->publish(grav_msg);

        // --- Broadcast TF: odom -> base_link ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = last_quat_[0];
        t.transform.rotation.y = last_quat_[1];
        t.transform.rotation.z = last_quat_[2];
        t.transform.rotation.w = last_quat_[3];
        tf_broadcaster_->sendTransform(t);

        // Periodic diagnostics
        if (read_count_ > 0 && read_count_ % 10000 == 0) {
            RCLCPP_INFO(get_logger(), "IMU: %lu reads, %lu errs, g=(%.1f,%.1f,%.1f)",
                        read_count_, error_count_, last_gravity_[0], last_gravity_[1], last_gravity_[2]);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bno085NodeCpp>());
    rclcpp::shutdown();
    return 0;
}
