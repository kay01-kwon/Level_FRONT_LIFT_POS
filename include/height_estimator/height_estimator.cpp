#include "height_estimator.hpp"

HeightEst::HeightEst()
{
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1.0;

    imu_subscriber = nh_.subscribe("/mavros/imu/data",
    1, &HeightEst::IMU_Callback, this);

    rp_extr_ptr = new RollPitchExtr();

}

void HeightEst::print_roll_pitch()
{
    double roll;
    double pitch;

    rp_extr_ptr->quat2rotm(q);

    rp_extr_ptr->get_roll_pitch(&roll, &pitch);

    cout<<"roll: "<<roll*180.0/M_PI<<",  "<<"pitch: "<<pitch*180.0/M_PI<<endl;

}

void HeightEst::IMU_Callback(const Imu::ConstPtr& imu_msg)
{
    q.x() = imu_msg->orientation.x;
    q.y() = imu_msg->orientation.y;
    q.z() = imu_msg->orientation.z;
    q.w() = imu_msg->orientation.w;

}

HeightEst::~HeightEst()
{
    delete rp_extr_ptr;
}