// Shaun Bowman
// 2021/10/15
// v0.0
// node name: av_pointcloud_node
// type: av_pointcloud

// * Purpose:
// 	* Drive automously mowbot through series of lat/lon points in sequence
// 	* Global 0 taken to be mowbot heading/position at start of controller action
// 	* Lookahead point, ie: kinematic path end point for planning purposes
// 	* Trajectory calculation
// 	* Conversion to turn radius desired
// 	* Command to turn_rad_arb
// 		*** av_turnrad_req** topic
// * Future:
// 	* Add throttle. At first, manual throttle
// 		* Slow for turns
// 	* Closed loop trajectory

/*
Series of lat lon pro
    x & y points
? Ability to reset Pose from Realsense?
--> perhaps create a rationalization of pose from RS

Calculate path length ?
Try to 

Idea's to try and generate drive path:
- RDP Algorithm to simplify path between points
- bezier curves?

*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <cmath>

void poset265Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // do nothing
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Read in parameters to global var's

    
    // Subscribe to input topics
    ros::Subscriber sub_pose = n.subscribe("t265_pose", 1000, poset265Callback);

    // Announce node is running
    ROS_INFO("turn_rad_arb node - RUNNING");

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}