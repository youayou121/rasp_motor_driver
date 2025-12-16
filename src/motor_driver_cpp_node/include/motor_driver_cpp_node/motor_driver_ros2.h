#pragma once
#include"motor_driver.h"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>

#include "velocity_smoother.h"     

class MotorDriverROS : public rclcpp::Node
{
public:
    MotorDriverROS(const std::string &name) : Node(name)
    {
        WHEEL_DISTANCE = 0.126;
        WHEEL_RADIUS = 0.024;
        TPR_L = 260 * 4;
        TPR_R = 260 * 4;
        control_hz_ = 20.0;

        x_ = y_ = theta_ = 0.0;
        vx_ = wz_ = 0.0;
        target_vx_ = target_wz_ = 0.0;

        last_left_ticks_ = 0;
        last_right_ticks_ = 0;

        motor_driver_ = std::make_shared<MotorDriver>();

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10,
            std::bind(&MotorDriverROS::cmd_callback, this, std::placeholders::_1)
        );

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / control_hz_)),
            std::bind(&MotorDriverROS::control_callback, this)
        );

        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorDriverROS::update_odom, this)
        );

        RCLCPP_INFO(get_logger(), "motor_driver_ros started");
    }

    ~MotorDriverROS()
    {
        motor_driver_->stop();
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_vx_ = msg->linear.x;
        target_wz_ = msg->angular.z;
    }

    void control_callback()
    {
        double output_vx = vx_;
        double output_wz = wz_;
        smoother_.update(vx_, wz_, target_vx_, target_wz_, 1.0 / control_hz_, output_vx, output_wz);

        double v_l = output_vx - output_wz * WHEEL_DISTANCE / 2.0;
        double v_r = output_vx + output_wz * WHEEL_DISTANCE / 2.0;

        vx_ = output_vx;
        wz_ = output_wz;

        motor_driver_->set_left_motor(v_l);
        motor_driver_->set_right_motor(v_r);
    }

    void update_odom()
    {
        int left_ticks, right_ticks;
        motor_driver_->get_ticks(left_ticks, right_ticks);
        printf("left_ticks: %d, right_ticks: %d\n", left_ticks, right_ticks);
        int delta_L = left_ticks - last_left_ticks_;
        int delta_R = right_ticks - last_right_ticks_;
        printf("delta_L: %d, delta_R: %d\n", delta_L, delta_R);
        last_left_ticks_  = left_ticks;
        last_right_ticks_ = right_ticks;

        double ds_L = 2 * M_PI * WHEEL_RADIUS * delta_L / TPR_L;
        double ds_R = 2 * M_PI * WHEEL_RADIUS * delta_R / TPR_R;

        double ds = (ds_L + ds_R) / 2.0;
        double dtheta = (ds_R - ds_L) / WHEEL_DISTANCE;

        x_ += ds * std::cos(theta_ + dtheta / 2.0);
        y_ += ds * std::sin(theta_ + dtheta / 2.0);
        theta_ += dtheta;

        normalize_angle(theta_);

        publish_odom();
        publish_tf();
    }

    void publish_odom()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom);
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    void normalize_angle(double &a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    double WHEEL_DISTANCE;
    double WHEEL_RADIUS;
    int TPR_L, TPR_R;

    double x_, y_, theta_;
    double vx_, wz_;
    double target_vx_, target_wz_;

    int last_left_ticks_;
    int last_right_ticks_;

    double control_hz_;

    std::shared_ptr<MotorDriver> motor_driver_;
    VelocitySmoother smoother_;
};
