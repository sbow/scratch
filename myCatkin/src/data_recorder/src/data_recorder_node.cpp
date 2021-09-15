/*
data_recorder.node.cpp
September 6, 2021
Shaun Bowman

Purpose:
- manage rosbag
*/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Gloabls from input topics
bool rosbagReq = false;

// Global state vars
bool newMsg = false;
bool isRecording = false;

void rosbagCallback(const std_msgs::Bool::ConstPtr& msg)
{
    rosbagReq = msg->data;
    newMsg = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(2);

    // Subscribe to input topics (from teleop_logi, and AV controllers)
    ros::Subscriber sub_rosbag = n.subscribe("/teleop_logi/rosbag_req", 1000, rosbagCallback);

    // Publish output topics (Road wheel angle cmd, Servo duty cmd)
    ros::Publisher pub_recordStart = n.advertise<std_msgs::Bool>("/record/start", 5);
    ros::Publisher pub_recordStop = n.advertise<std_msgs::Bool>("/record/stop", 5);
    std_msgs::Bool msgSend;
    msgSend.data = true;

    // Announce node is running
    ROS_INFO("data_recorder node - RUNNING");

    while (ros::ok())
    {
        if (newMsg)
        {
            // new msg flag indicates new rosbag request
            if (rosbagReq && not isRecording)
            {
                pub_recordStart.publish(msgSend);
                isRecording = true;
            }
            else if(isRecording && not rosbagReq)
            {
                pub_recordStop.publish(msgSend);
                isRecording = false;
            }
            newMsg = false; // for debounc
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}