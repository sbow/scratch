// Shaun Bowman
// Aug 8 2021
// Purpose:
//  Publisher ROS node for tracking camera
// Uses:
//  Intel REALSENSE T265 Tracking camera
// Tested on:
//  Ros Noetic, Ubuntu 20.04

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <algorithm>

// Find devices with specified streams
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }
        }
        if (found_all_streams)
            return true;
    }
    // After scanning all devices, not all requested streams were found
    for (auto& type : unavailable_streams)
    {
        switch (type)
        {
        case RS2_STREAM_POSE:
        case RS2_STREAM_FISHEYE:
            std::cerr << "Connect T26X and rerun the demo" << std::endl;
            break;
        case RS2_STREAM_DEPTH:
            std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
            break;
        case RS2_STREAM_COLOR:
            std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
            break;
        default:
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
        }
    }
    return false;
}

int main(int argc, char * argv[]) try
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "poseTx");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher t265_pose_pub = n.advertise<geometry_msgs::Pose>("t265_pose", 1000);
  ros::Publisher t265_twist_pub = n.advertise<geometry_msgs::Twist>("t265_twist", 1000);
  ros::Publisher t265_confTracker_pub = n.advertise<std_msgs::Int16>("t265_confTracker", 1000);
  ros::Publisher t265_confMapper_pub = n.advertise<std_msgs::Int16>("t265_confMapper", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  /**
   * ---Intel realsense initialization code
   */

    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    ROS_INFO("Intel Realsesne T265 tracking camera initialized");
  /**
   * END intel realsense init code
   */

  while (ros::ok())
  {
    /**
     * ---INTEL Realsense interface code---
     */

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    
    // Get some data from the T265:
    float dX = pose_data.translation.x;
    float dY = pose_data.translation.y;
    float dZ = pose_data.translation.z;
    float qW = pose_data.rotation.w;
    float qX = pose_data.rotation.x;
    float qY = pose_data.rotation.y;
    float qZ = pose_data.rotation.z;
    float vX = pose_data.velocity.x;
    float vY = pose_data.velocity.y;
    float vZ = pose_data.velocity.z;
    float avX = pose_data.angular_velocity.x; // angular velocity
    float avY = pose_data.angular_velocity.y; // angular velocity
    float avZ = pose_data.angular_velocity.z; // angular velocity
    /**< Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High */
    unsigned int confTracker = pose_data.tracker_confidence; 
    unsigned int confMapper = pose_data.mapper_confidence;

    // Output some sample data to std out:
    //std::cout << "\r" << "Device rotation w: " << std::setprecision(3) << std::fixed << qW << "\n";
    //std::cout << "\r" << "Device rotation x: " << std::setprecision(3) << std::fixed << qX << "\n";

    /**
     * ---ROS Publisher code---
     */
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    // Output some pose data to chatter topic
    ss << "Quaterion X: " << std::setprecision(3) << std::fixed << qX;
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);


    /** Populate & publish geometry_msgs/Pose type message under t265_pose topic
     */
    geometry_msgs::Pose pose;
    pose.position.x = dX;
    pose.position.y = dY;
    pose.position.z = dZ;
    pose.orientation.x = qX;
    pose.orientation.y = qY;
    pose.orientation.z = qZ;
    pose.orientation.w = qW;
    t265_pose_pub.publish(pose);

    /** Populate & publish geometry_msgs/Twist type message (lin/ang vel) 
     * under the t265_twist topic
     */
    geometry_msgs::Twist twist;
    twist.linear.x = vX;
    twist.linear.y = vY;
    twist.linear.z = vZ;
    twist.angular.x = avX;
    twist.angular.y = avY;
    twist.angular.z = avZ;
    t265_twist_pub.publish(twist);

    /** Populate & publish tracking & mapping confidence measurements (0 fail, 3 max)
     */
    std_msgs::Int16 trackConf;
    std_msgs::Int16 mapConf;
    trackConf.data = confTracker;
    mapConf.data = confMapper;
    t265_confTracker_pub.publish(trackConf);
    t265_confMapper_pub.publish(mapConf);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
