#ifndef HEIGHT_ESTIMATOR_HPP
#define HEIGHT_ESTIMATOR_HPP

#include "roll_pitch_extractor.hpp"
#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

using std::cout;
using std::endl;

using sensor_msgs::Imu;
using std_msgs::Float64;
using std_msgs::Int32;


class HeightEst{

    public:
        HeightEst();

        void print_roll_pitch();

        ~HeightEst();

    private:

        void IMU_Callback(const Imu::ConstPtr& imu_msg);

        ros::NodeHandle nh_;
        ros::Subscriber imu_subscriber;
        ros::Publisher h_est_publisher;
        ros::Publisher des_q_publisher;
        

        quat q;
        double h_hat;
        int32_t q_lift_des;

        RollPitchExtr* rp_extr_ptr;

};

#endif