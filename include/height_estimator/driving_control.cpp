#include "driving_control.hpp"

DriverCtrl::DriverCtrl()
{
    setup();
}

void DriverCtrl::setup()
{

    converter_ptr = new Converter();

    nh_.getParam("offset0",offset_(0));
    nh_.getParam("offset1",offset_(1));
    nh_.getParam("offset2",offset_(2));

    nh_.getParam("wheel_sep1",wheel_sep_1);
    nh_.getParam("wheel_sep2",wheel_sep_2);
    nh_.getParam("q_lift_fix",q_lift_set);

    wheel_radius = 0.169/2.0;
    radps2rpm = 2.0*M_PI/60.0;

    for(int i = 0; i < 3; i++)
        q_lift_inc[i] = converter_ptr->convert_q_lift_des2inc(q_lift_set,offset_(i));

    twist_vel_subscriber = nh_.subscribe("/cmd_vel",
    1,
    &DriverCtrl::callback_twist, this);

    target_publisher = nh_.advertise<target>("/target",1);
    target_dxl_publisher = nh_.advertise<target_dxl>("/target_dxl",1);
}

void DriverCtrl::twist2wheel_vel()
{

    v_f << 1,2;

    double v_f_mag, v_l_mag, v_r_mag;
    double theta_f, theta_l, theta_r;
    
    v_f_mag = sqrt(v_f.transpose()*v_f);

    wheel_vel_steering_pos << v_f_mag, v_l_mag, v_r_mag,
    theta_f, theta_l, theta_r;


}

void DriverCtrl::steering_pos2dxl_inc()
{
    steering_pos_inc[0] = (int32_t)(2048.0 + 
    steering_pos(0)*N_STEERING*4096.0/360.0);

    steering_pos_inc[1] = (int32_t) (2048.0 - 60.0*N_STEERING*4096.0/360.0);
    steering_pos_inc[2] = (int32_t) (2048.0 + 60.0*N_STEERING*4096.0/360.0);
}

void DriverCtrl::publish_values()
{
    target target_msg;
    target_dxl target_dxl_msg;

    for(int i = 0; i < 3; i++)
    {
        target_msg.target_LIFT[i] = q_lift_inc[i];
        target_msg.target_PAN[i] = 0;
        target_msg.target_WHEEL[i] = wheel_vel[i];
        target_dxl_msg.target_dxl[i] = steering_pos_inc[i];
    }
}

void DriverCtrl::callback_twist(const Twist::ConstPtr& twist_msg)
{
    linear_vel_x = twist_msg->linear.x;
    linear_vel_y = twist_msg->linear.y;
    angular_vel = twist_msg->angular.z;
}

DriverCtrl::~DriverCtrl()
{
    delete converter_ptr;
}