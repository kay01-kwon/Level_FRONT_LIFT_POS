#include "height_estimator.hpp"

HeightEst::HeightEst()
{
    // Initialize quaternion of imu
    q_.x() = 0;
    q_.y() = 0;
    q_.z() = 0;
    q_.w() = 1.0;

    // Get offset position of lift motor from launch file
    nh_.getParam("offset0",offset_(0));
    nh_.getParam("offset1",offset_(1));
    nh_.getParam("offset2",offset_(2));

    cout<<"Offset position of lift position\n";

    for(int i = 0; i < 3; i++)
        cout<<offset_(i)<<" ";
    
    std::cout<<"\n";


    imu_subscriber = nh_.subscribe("/mavros/imu/data",
    1, &HeightEst::IMU_Callback, this);

    motor_subscriber = nh_.subscribe("/actual",
    1,&HeightEst::motor_Callback, this);

    h_est_publisher = nh_.advertise<Float64>("/h_est",10);
    q_des_publisher = nh_.advertise<Float64>("/q_lift_des",10);
    rp_publisher = nh_.advertise<rp>("/rp_result",10);

    // Dynamic memory allocation
    rp_extr_ptr = new RollPitchExtr();
    coordTf_ptr = new CoordTf();
    converter_ptr = new Converter(offset_);
}

void HeightEst::print_roll_pitch()
{

    get_roll_pitch();

    cout<<"roll: "<<phi*180.0/M_PI<<",  ";
    cout<<"pitch: "<<theta*180.0/M_PI<<endl;

}

void HeightEst::publish_values()
{
    Float64 height_msg;
    Float64 q_des_msg;
    rp rp_msg;

    cout<<"roll: "<<phi*180.0/M_PI<<",  ";
    cout<<"pitch: "<<theta*180.0/M_PI<<endl;

    get_roll_pitch();
    h_hat_ = get_height();
    q_lift_des_ = get_lift_des();

    cout<<"Estimated height: "<<h_hat_<<endl;
    cout<<"Corrected lift position: "<<q_lift_des_<<endl;

    height_msg.data = h_hat_;
    q_des_msg.data = q_lift_des_;

    rp_msg.theta = theta*180.0/M_PI;
    rp_msg.phi = phi*180.0/M_PI;

    h_est_publisher.publish(height_msg);
    q_des_publisher.publish(q_des_msg);
    rp_publisher.publish(rp_msg);
}

void HeightEst::get_roll_pitch()
{

    rp_extr_ptr->quat2rotm(q_);
    rp_extr_ptr->get_roll_pitch(&phi, &theta);

}

double HeightEst::get_height()
{

    double Dx12, Dy12, Dz12;
    double h_;
    coordTf_ptr->setPos(q_pan_, q_lift_);

    for(int i = 0; i < 3; i++)
        coordTf_ptr->getWheelCenter(i, b_t_wc);

    Dx12 = b_t_wc(0,0) - b_t_wc(0,1);
    Dy12 = b_t_wc(1,0) - b_t_wc(1,1);
    Dz12 = b_t_wc(2,0) - b_t_wc(2,1);
    
    h_ = -Dx12*sin(theta) 
    + Dy12*cos(theta)*sin(phi)
    + Dz12*cos(theta)*cos(phi);

    return h_;

}

double HeightEst::get_lift_des()
{
    double l, c;
    l = 0.400;
    c = 0.170;

    int32_t q_lift_des_inc;

    q_lift_des_ = acos((-b_t_wc(2,1) - h_hat_ - c)/l)*180.0/M_PI;

    return q_lift_des_;
}


void HeightEst::IMU_Callback(const Imu::ConstPtr& imu_msg)
{
    q_.x() = imu_msg->orientation.x;
    q_.y() = imu_msg->orientation.y;
    q_.z() = imu_msg->orientation.z;
    q_.w() = imu_msg->orientation.w;

}

void HeightEst::motor_Callback(const actual::ConstPtr& motor_msg)
{
    for(int i = 0; i < 3; i++)
    {
        q_pan_(i) = 
        converter_ptr->convert_actaul2q_pan(motor_msg->act_PAN_pos[i]);
        
        q_lift_(i) = 
        converter_ptr->convert_actual2q_lift(motor_msg->act_LIFT_pos[i],offset_(i));
    }

}

HeightEst::~HeightEst()
{
    delete rp_extr_ptr;
    delete coordTf_ptr;
    delete converter_ptr;
}