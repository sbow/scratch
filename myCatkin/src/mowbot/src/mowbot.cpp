
#include <ros/ros.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mowbot");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    // Announce node is running
    ROS_INFO("mowbot node - RUNNING");



    while (ros::ok())
    {
        // loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}