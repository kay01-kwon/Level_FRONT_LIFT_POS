#include "driver_control.hpp"

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

    nh_.getParam("H",H);
    nh_.getParam("L",L);
    nh_.getParam("q_lift_fix",q_lift_set);

    linear_vel_x = 0;
    linear_vel_y = 0;
    angular_vel = 0;
    rho = 0;

    wheel_radius = 0.169/2.0;
    radps2rpm = 60.0/2.0/M_PI;

    deg2inc = 4096.0/360.0;
    rad2deg = 180.0/M_PI;

    enable_control = false;

    for(int i = 0; i < 3; i++)
        q_lift_inc[i] = converter_ptr->convert_q_lift_des2inc(q_lift_set,offset_(i));


    twist_vel_subscriber = nh_.subscribe("/cmd_vel",
    1,
    &DriverCtrl::callback_twist, 
    this);

    target_publisher = nh_.advertise<target>("/target",1);
    target_dxl_publisher = nh_.advertise<target_dxl>("/target_dxl",1);
}

void DriverCtrl::get_rho()
{

    rho = linear_vel_x/angular_vel;

}

void DriverCtrl::get_wheel_linear_vel()
{
    double rho1, rho2, rho3;

    rho1 = sqrt(pow(rho,2) + pow(2.0/3.0*H,2));
    rho2 = sqrt(pow(rho-0.5*L,2) + pow(1.0/3.0*H,2));
    rho3 = sqrt(pow(rho+0.5*L,2) + pow(1.0/3.0*H,2));
    
    wheel_linear_vel << rho1*fabs(angular_vel), 
                        rho2*fabs(angular_vel), 
                        rho3*fabs(angular_vel);

    if(linear_vel_x < 0)
        wheel_linear_vel = - wheel_linear_vel;

}

void DriverCtrl::twist2wheel_vel_steer()
{

    // Angular velocity check
    if(fabs(angular_vel) < 0.001)
    {
        steering_pos_inc[0] = (int32_t) 2048;

        steering_pos_inc[1] = (int32_t) (2048.0 + 60.0*N_STEERING*deg2inc);
        steering_pos_inc[2] = (int32_t) (2048.0 - 60.0*N_STEERING*deg2inc);

        for(int i = 0; i < 3; i++)
        {
            wheel_linear_vel(i) = linear_vel_x;
            wheel_vel_radps[i] = wheel_linear_vel(i)/wheel_radius;
        }

    }else
    {
        get_rho();

        steering_pos << atan(2.0*H/3.0/rho)*180.0/M_PI,
        (M_PI/3.0 - atan(H/3.0/(rho - 0.5 * L)))*180.0/M_PI,
        -( M_PI/3.0 + atan(H/3.0/(rho + 0.5 * L)))*180.0/M_PI;

        get_wheel_linear_vel();

        wheel_vel_radps = wheel_linear_vel / wheel_radius;

        for(int i = 0; i < 3; i++)
            steering_pos_inc[i] = (int32_t) (2048.0 
            + N_STEERING*deg2inc*steering_pos(i));
    }

    for(int i = 0; i < 3;i++)
        wheel_vel_rpm[i] = (int32_t) -wheel_vel_radps(i) * radps2rpm;
    
    wheel_vel_rpm[0] = -wheel_vel_rpm[0];


    cout<<"Steering position: ";
    
    for(int i = 0; i < 3; i++)
        cout<<steering_pos(i)<<", ";
    
    cout<<endl;

    cout<<"Wheel linear velocity: ";

    for(int i = 0; i < 3; i++)
        cout<<wheel_linear_vel(i)<<", ";
    
    cout<<endl;

}

void DriverCtrl::publish_values()
{
    target target_msg;
    target_dxl target_dxl_msg;

    twist2wheel_vel_steer();

    for(int i = 0; i < 3; i++)
    {
        target_msg.target_LIFT[i] = q_lift_inc[i];
        target_msg.target_PAN[i] = 0;
        target_msg.target_WHEEL[i] = wheel_vel_rpm[i];
        target_dxl_msg.target_dxl[i] = steering_pos_inc[i];
    }

    target_publisher.publish(target_msg);
    target_dxl_publisher.publish(target_dxl_msg);
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