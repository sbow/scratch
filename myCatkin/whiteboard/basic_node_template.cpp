
#include <ros/ros.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Announce node is running
    ROS_INFO("turn_rad_arb node - RUNNING");

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}