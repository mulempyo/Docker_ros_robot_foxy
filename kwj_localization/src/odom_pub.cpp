#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

// Constants
const double PI = 3.141592;
const double WHEEL_DIAMETER = 0.067; // Wheel diameter in meters
const double WHEEL_BASE = 0.212; // Center of left tire to center of right tire
const double LEFT_TICKS_PER_REVOLUTION = 1700;
const double RIGHT_TICKS_PER_REVOLUTION = 1800;

// Variables
double distanceLeft = 0.0, distanceRight = 0.0;
double x = 0.0, y = 0.0, th = 0.0;
double vx = 0.0, vth = 0.0;
int leftTicks = 0, rightTicks = 0;
rclcpp::Time last_time;

// Create odometry data publisher
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;

void Calc_Left(const std_msgs::msg::Int32::SharedPtr leftCount) {
    static int lastCountL = 0;
    leftTicks = leftCount->data - lastCountL;

    // Handle overflow/underflow
    if (leftTicks > 2147483000) {
        leftTicks = 0 - (4294967295 - leftTicks);
    } else if (leftTicks < -2147483000) {
        leftTicks = 4294967295 - leftTicks;
    }

    distanceLeft = (static_cast<double>(leftTicks) / LEFT_TICKS_PER_REVOLUTION) * PI * WHEEL_DIAMETER;
    lastCountL = leftCount->data;
}

void Calc_Right(const std_msgs::msg::Int32::SharedPtr rightCount) {
    static int lastCountR = 0;
    rightTicks = rightCount->data - lastCountR;

    // Handle overflow/underflow
    if (rightTicks > 2147483000) {
        rightTicks = 0 - (4294967295 - rightTicks);
    } else if (rightTicks < -2147483000) {
        rightTicks = 4294967295 - rightTicks;
    }

    distanceRight = (static_cast<double>(rightTicks) / RIGHT_TICKS_PER_REVOLUTION) * PI * WHEEL_DIAMETER;
    lastCountR = rightCount->data;
}

void update_odom(rclcpp::Node::SharedPtr node) {
    auto current_time = node->now();
    double dt = (current_time - last_time).seconds();

    double left_velocity = distanceLeft / dt;
    double right_velocity = distanceRight / dt;
    double averageVelocity = (left_velocity + right_velocity) / 2;

    double dist = (distanceRight + distanceLeft) / 2;
    double dth = (distanceRight - distanceLeft) / WHEEL_BASE;

    vx = averageVelocity;
    vth = dth / dt;

    double dx = dist * cos(th);
    double dy = dist * sin(th);

    x += dx;
    y += dy;
    th += dth;

    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, th));

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    for (int i = 0; i < 36; i++) {
        if (i == 0 || i == 7 || i == 14) {
            odom.pose.covariance[i] = 0.01;
        } else if (i == 21 || i == 28 || i == 35) {
            odom.pose.covariance[i] = 0.1;
        } else {
            odom.pose.covariance[i] = 0.0;
        }
    }

    odom_data_pub_quat->publish(odom);
    last_time = current_time;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odom_pub");

    odom_data_pub_quat = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    auto subForRightCounts = node->create_subscription<std_msgs::msg::Int32>(
        "right_ticks", 100, Calc_Right);
    auto subForLeftCounts = node->create_subscription<std_msgs::msg::Int32>(
        "left_ticks", 100, Calc_Left);

    rclcpp::Rate r(30); // 30Hz
    last_time = node->now();

    while (rclcpp::ok()) {
        update_odom(node);
        rclcpp::spin_some(node);
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

