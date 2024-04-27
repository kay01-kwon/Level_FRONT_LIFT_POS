#ifndef HEIGHT_ESTIMATOR_HPP
#define HEIGHT_ESTIMATOR_HPP

#include "roll_pitch_extractor.hpp"
#include "coordinate_transform.hpp"
#include "converter.hpp"

#include <ros/ros.h>
#include <iostream>

#include <msg_pkg/actual.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float64.h>

#include <height_estimator/rp.h>

using std::cout;
using std::endl;

using msg_pkg::actual;
using sensor_msgs::Imu;

using std_msgs::Float64;

using height_estimator::rp;

class HeightEst{

    public:
        HeightEst();

        void print_roll_pitch();

        void publish_values();

        void get_roll_pitch();

        double get_height();

        double get_lift_des();

        ~HeightEst();

    private:

        void IMU_Callback(const Imu::ConstPtr& imu_msg);
        
        void motor_Callback(const actual::ConstPtr& motor_msg);

        ros::NodeHandle nh_;

        ros::Subscriber imu_subscriber;
        ros::Subscriber motor_subscriber;

        ros::Publisher h_est_publisher;
        ros::Publisher q_des_publisher;
        ros::Publisher rp_publisher;

        double phi, theta;

        mat31 offset_;
        mat31 q_pan_;
        mat31 q_lift_;

        mat43 b_t_wc;

        quat q_;
        double h_hat_;
        int32_t q_lift_des_;

        RollPitchExtr* rp_extr_ptr;
        CoordTf* coordTf_ptr;
        Converter* converter_ptr;
        

};

#endif