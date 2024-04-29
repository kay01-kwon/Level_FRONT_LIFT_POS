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

void AttCtrl::setup()
{
    // Initialize quaternion of imu
    q_.x() = 0;
    q_.y() = 0;
    q_.z() = 0;
    q_.w() = 1.0;

    nh_.getParam("q_lift_fix",q_lift_fix);
    nh_.getParam("Kp",Kp);

    nh_.getParam("offset0",offset_(0));
    nh_.getParam("offset1",offset_(1));
    nh_.getParam("offset2",offset_(2));

    for(int i = 0; i < 3; i++)
    {
        wheel_vel[i] = 0;
        q_lift_actual(i) = offset_(i);
    }


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

double AttCtrl::get_rear_lift_pos(int i)
{
    double q_lift_rear;
    q_lift_rear = q_lift_actual(i+1) + Kp*theta;
    return q_lift_rear;
}

void AttCtrl::publish_values()
{
    int32_t q_lift_des_inc[3];
    double q_lift_rear[2];

    rp rp_msg;

    get_pitch();

    q_lift_rear[0] = get_rear_lift_pos(0);
    q_lift_rear[1] = get_rear_lift_pos(1);

    q_lift_des_inc[0] = converter_ptr->convert_q_lift_des2inc(q_lift_fix, offset_(0));
    
    for(int i = 0; i < 2; i++)
        q_lift_des_inc[i+1] = converter_ptr->convert_q_lift_des2inc(q_lift_rear[i], offset_(i+1));

    for(int i = 0; i < 3; i++)
    {
        target_msg.target_LIFT[i] = q_lift_des_inc[i];
        target_msg.target_PAN[i] = 0;
        target_msg.target_WHEEL[i] = wheel_vel[i];
    }

    target_dxl_msg.target_dxl[0] = 2048;
    target_dxl_msg.target_dxl[1] = (int32_t) -70.0/40.0*4096.0/360.0*60.0;
    target_dxl_msg.target_dxl[2] = (int32_t) 70.0/40.0*4096.0/360.0*60.0;

    rp_msg.phi = phi*180.0/M_PI;
    rp_msg.theta = theta*180.0/M_PI;

    target_publisher.publish(target_msg);
    target_dxl_publisher.publish(target_dxl_msg);
    rp_publisher.publish(rp_msg);
}

void AttCtrl::callback_imu(const Imu::ConstPtr& imu_msg)
{

    q_.x() = imu_msg->orientation.x;
    q_.y() = imu_msg->orientation.y;
    q_.z() = imu_msg->orientation.z;
    q_.w() = imu_msg->orientation.w;
}

void AttCtrl::callback_twist(const Twist::ConstPtr& twist_msg)
{
    for(int i = 0; i < 3; i++)
        wheel_vel[i] = -(twist_msg->linear.x*0.169*60.0/M_PI);
    
    wheel_vel[0] = -wheel_vel[0];

}

void AttCtrl::callback_actual(const actual::ConstPtr& actual_msg)
{
    int32_t q_lift_actual_inc[3];
    for(int i = 0; i < 3; i++)
    {
        q_lift_actual_inc[i] = actual_msg->act_LIFT_pos[i];
        
        q_lift_actual(i) = converter_ptr->convert_actual2q_lift(q_lift_actual_inc[i], offset_(i));
    }
    
}

AttCtrl::~AttCtrl()
{
    delete rp_extr_ptr;
    delete converter_ptr;
}