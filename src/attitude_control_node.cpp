#include <height_estimator/attitude_control.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "attitude_control_node");

    AttCtrl attitude_ctrl;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        attitude_ctrl.publish_values();
        loop_rate.sleep();
    }
}