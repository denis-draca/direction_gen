#include <ros/ros.h>
#include "a_start/main_path.h"

int main(int argc, char** argv)
{
    ros::init( argc, argv, "main_path");

    ros::NodeHandle n;

    main_path path(n);

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        path.find_steps();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("****MAIN_PATH SHUTTING DOWN****");
}
