#include <height_estimator/driver_control.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "driver_node");
    DriverCtrl driver_control_node;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        driver_control_node.publish_values();
        loop_rate.sleep();
    }

}