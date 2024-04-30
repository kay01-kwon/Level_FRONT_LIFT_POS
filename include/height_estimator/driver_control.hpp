#ifndef DRIVER_CONTROL_HPP
#define DRIVER_CONTROL_HPP

#include "converter.hpp"

#include <ros/ros.h>
#include <iostream>

#include <msg_pkg/target.h>
#include <msg_pkg/target_dxl.h>
#include <geometry_msgs/Twist.h>

using msg_pkg::target;
using msg_pkg::target_dxl;

using geometry_msgs::Twist;


class DriverCtrl{

    public:

        DriverCtrl();

        void setup();

        void get_rho();

        void get_wheel_linear_vel();

        void twist2wheel_vel_steer();

        void publish_values();

        ~DriverCtrl();

    private:

    ros::NodeHandle nh_;
    
    ros::Subscriber twist_vel_subscriber;

    ros::Publisher target_publisher;
    ros::Publisher target_dxl_publisher;

    Converter* converter_ptr;

    void callback_twist(const Twist::ConstPtr& twist_msg);

    mat31 offset_, steering_pos, wheel_linear_vel;
    mat31 wheel_vel_radps;

    double q_lift_set;

    int32_t q_lift_inc[3];
    int32_t wheel_vel_rpm[3];
    int32_t steering_pos_inc[3];

    double H, L;

    double linear_vel_x, linear_vel_y, angular_vel, rho;

    double wheel_radius, radps2rpm;

    double deg2inc, rad2deg;

};

#endif