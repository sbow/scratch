/*
data_recorder.node.cpp
September 6, 2021
Shaun Bowman

Purpose:
- manage rosbag
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(2);

    // Read in parameters to global var's
    bool gotParam = true;
    //gotParam = gotParam && n.getParam("servo_right", k_servo_right);
    if (not gotParam)
        ROS_FATAL("Failed to load l_wheelbase / track / min turn rad / servo parameters");

    // Subscribe to input topics (from teleop_logi, and AV controllers)
    //ros::Subscriber sub_teleSteer = n.subscribe("/teleop_logi/tele_steer_req", 1000, teleSteerCallback);

    // Publish output topics (Road wheel angle cmd, Servo duty cmd)
    // ros::Publisher pub_rwaRadiansCmd = n.advertise<std_msgs::Float32>("/turn_arb/rwaRadiansCmd",5);
    // std_msgs::Float32 rwaRadiansCmd;

    // Announce node is running
    ROS_INFO("data_recorder node - RUNNING");

    // Debug:

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}