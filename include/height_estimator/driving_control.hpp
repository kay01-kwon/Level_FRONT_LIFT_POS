#ifndef DRIVING_CONTROL_HPP
#define DRIVING_CONTROL_HPP

#include "converter.hpp"

#include <ros/ros.h>
#include <iostream>

#include <msg_pkg/target.h>
#include <msg_pkg/target_dxl.h>
#include <geometry_msgs/Twist.h>

using msg_pkg::target;
using msg_pkg::target_dxl;

using geometry_msgs::Twist;

typedef Eigen::Matrix<double,6,2> mat62;
typedef Eigen::Matrix<double,6,1> mat61;
typedef Eigen::Matrix<double,2,1> mat21;

class DriverCtrl{

    public:

        DriverCtrl();

        void setup();

        void twist2wheel_vel();

        void steering_pos2dxl_inc();

        void publish_values();

        ~DriverCtrl();

    private:

    ros::NodeHandle nh_;
    
    ros::Subscriber twist_vel_subscriber;

    ros::Publisher target_publisher;
    ros::Publisher target_dxl_publisher;

    Converter* converter_ptr;

    void callback_twist(const Twist::ConstPtr& twist_msg);

    mat31 offset_, steering_pos;

    double q_lift_set;

    mat21 twist;


    mat21 v_f, v_l, v_r;

    mat61 wheel_vel_steering_pos;

    mat62 transform;

    int32_t q_lift_inc[3];
    int32_t wheel_vel[3];
    int32_t steering_pos_inc[3];

    double wheel_sep_1, wheel_sep_2;

    double linear_vel_x, linear_vel_y;
    double angular_vel; 

    double wheel_radius, radps2rpm;

};



#endif