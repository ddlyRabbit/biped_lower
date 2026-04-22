#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "biped_driver_cpp/im10a_reader.hpp"

class Im10aNode : public rclcpp::Node {
public:
    Im10aNode() : Node("im10a_node") {
        // Parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 460800);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<double>("rate_hz", 200.0);

        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        double rate_hz = this->get_parameter("rate_hz").as_double();

        RCLCPP_INFO(this->get_logger(), "Starting IM10A Node on port %s at %d baud", port.c_str(), baud);

        if (!reader_.init(port, baud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IM10A IMU");
            return;
        }

        // QoS for sensor data
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
        gravity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/gravity", qos);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(period, std::bind(&Im10aNode::timer_callback, this));
    }

private:
    void timer_callback() {
        ImuData data = reader_.read();

        auto now = this->now();

        // Publish IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = frame_id_;
        imu_msg.orientation.x = data.quaternion[0];
        imu_msg.orientation.y = data.quaternion[1];
        imu_msg.orientation.z = data.quaternion[2];
        imu_msg.orientation.w = data.quaternion[3];
        imu_msg.angular_velocity.x = data.gyro[0];
        imu_msg.angular_velocity.y = data.gyro[1];
        imu_msg.angular_velocity.z = data.gyro[2];
        // Note: Raw acceleration is currently not being output by the reader for simplicity,
        // so we don't populate linear_acceleration here.
        imu_pub_->publish(imu_msg);

        // Publish Gravity message
        geometry_msgs::msg::Vector3Stamped gravity_msg;
        gravity_msg.header.stamp = now;
        gravity_msg.header.frame_id = frame_id_;
        gravity_msg.vector.x = data.gravity[0];
        gravity_msg.vector.y = data.gravity[1];
        gravity_msg.vector.z = data.gravity[2];
        gravity_pub_->publish(gravity_msg);

        // Broadcast TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = data.quaternion[0];
            t.transform.rotation.y = data.quaternion[1];
            t.transform.rotation.z = data.quaternion[2];
            t.transform.rotation.w = data.quaternion[3];
            tf_broadcaster_->sendTransform(t);
        }
    }

    Im10aReader reader_;
    std::string frame_id_;
    bool publish_tf_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Im10aNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}