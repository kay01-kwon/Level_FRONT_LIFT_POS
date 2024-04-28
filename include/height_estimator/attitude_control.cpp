#include "attitude_control.hpp"

AttCtrl::AttCtrl()
{
    offset_ << 84.2608, 84.0448, 84.1169;

    setup();
}

AttCtrl::AttCtrl(mat31 offset)
{
    offset_ = offset;

    nh_.getParam("offset0",offset_(0));
    nh_.getParam("offset1",offset_(1));
    nh_.getParam("offset2",offset_(2));

    setup();
}

void AttCtrl::setup(int i)
{
    nh_.getParam("q_lift_fix",q_lift_fix);
    nh_.getParam("Kp",Kp);

    /**
     * Subscriber setup
    */
    // 1. Imu subscriber
    imu_subscriber = nh_.subscribe("/mavros/imu/data",
    1, &AttCtrl::callback_imu, this);

    // 2. twist subscriber
    twist_subscriber = nh_.subscribe("/cmd_vel",
    1, &AttCtrl::callback_twist, this);

    /**
     * Publisher setup
    */
    // 1. target_publihser
    target_publisher = nh_.advertise<target>("/target",10);
    
    // 2. target_dxl_publisher
    target_dxl_publisher = nh_.advertise<target_dxl>("/target_dxl",10);
    
    // 3. rp_publisher
    rp_publisher = nh_.advertise<rp>("/rp_result",10);

    rp_extr_ptr = new RollPitchExtr();
    converter_ptr = new Converter();

}

void AttCtrl::get_pitch()
{
    rp_extr_ptr->quat2rotm(q_);
    rp_extr_ptr->get_roll_pitch(&phi, &theta);
    
}

double AttCtrl::get_rear_lift_pos()
{
    double q_lift_rear;
    q_lift_rear = q_lift_fix - Kp*theta;
}

void AttCtrl::publish_values()
{

}

void AttCtrl::callback_imu(Imu::ConstPtr& imu_msg)
{
    q_.x() = imu_msg->orientation.x;
    q_.y() = imu_msg->orientation.y;
    q_.z() = imu_msg->orientation.z;
    q_.w() = imu_msg->orientation.w;


}

void AttCtrl::callback_twist(Twist::ConstPtr& twist_msg)
{
    for(int i = 0; i < 3; i++)
        target_msg.target_WHEEL[i] = 
        twist_msg->linear.x*0.169*60.0/M_PI;
}