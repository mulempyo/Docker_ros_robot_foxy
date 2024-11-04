#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("cmd_vel_to_odom"), x(0), y(0), th(0), vx(0), vth(0), imu_yaw_rate(0.0), WHEEL_BASE(0.212) {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OdometryNode::cmdCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 100, std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

        last_time_ = this->get_clock()->now();
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), std::bind(&OdometryNode::update_odom, this));

    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_ = *msg;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_yaw_rate = msg->angular_velocity.z;
    }

    void update_odom() {
    auto current_time = this->get_clock()->now();
    dt = (current_time - last_time_).seconds();

    double left_velocity = cmd_vel_.linear.x - (cmd_vel_.angular.z * WHEEL_BASE / 2.0);
    double right_velocity = cmd_vel_.linear.x + (cmd_vel_.angular.z * WHEEL_BASE / 2.0);

    double dist = (left_velocity + right_velocity) / 2.0 * dt;
    double dth = imu_yaw_rate * dt;

    double dx = dist * cos(th);
    double dy = dist * sin(th);

    x += dx;
    y += dy;
    th += dth;
    th = fmod(th + 2 * M_PI, 2 * M_PI);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, th);  // Roll, Pitch, Yaw

    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = (dt > 0) ? dist / dt : 0.0;
    odom_msg.twist.twist.angular.z = (dt > 0) ? dth / dt : 0.0;

    for (int i = 0; i < 36; i++) {
        if (i == 0 || i == 7 || i == 14) {
            odom_msg.pose.covariance[i] = 0.01;
        } else if (i == 21 || i == 28 || i == 35) {
            odom_msg.pose.covariance[i] = 0.1;
        } else {
            odom_msg.pose.covariance[i] = 0.0;
        }
    }

    odom_pub_->publish(odom_msg);
    last_time_ = current_time;
}


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_vel_;
    rclcpp::Time last_time_;

    double x, y, th;
    double dt;
    double vx, vth;
    double imu_yaw_rate;
    const double WHEEL_BASE;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}

