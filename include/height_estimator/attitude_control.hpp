#ifndef ATTITUDE_CONTROL_HPP
#define ATTITUDE_CONTROL_HPP

#include "roll_pitch_extractor.hpp"
#include "converter.hpp"

#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include <msg_pkg/target.h>
#include <msg_pkg/target_dxl.h>
#include <height_estimator/rp.h>

using std::cout;
using std::endl;

using sensor_msgs::Imu;
using geometry_msgs::Twist;

using msg_pkg::target;
using msg_pkg::target_dxl;
using height_estimator::rp;

class AttCtrl{

    public:
        
        AttCtrl();

        AttCtrl(mat31 offset);

        void setup();

        void get_pitch();

        double get_rear_lift_pos();

        void publish_values();

        ~AttCtrl();


    private:

        ros::NodeHandle nh_;

        ros::Subscriber imu_subscriber;
        ros::Subscriber twist_subscriber;

        ros::Publisher target_publisher;
        ros::Publisher target_dxl_publisher;
        ros::Publisher rp_publisher;

        RollPitchExtr* rp_extr_ptr;
        Converter* converter_ptr;

        void callback_imu(const Imu::ConstPtr& imu_msg);
        void callback_twist(const Twist::ConstPtr& twist_msg);

        double q_lift_fix;

        target target_msg;
        target_dxl target_dxl_msg;

        mat31 offset_;
        double Kp;

        quat q_;
        double theta, phi;

        int32_t wheel_vel[3];

};


#endif