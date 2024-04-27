#include <height_estimator/height_estimator.hpp>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"height_est_node");

    HeightEst h_est;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        h_est.print_roll_pitch();
        loop_rate.sleep();
    }
}