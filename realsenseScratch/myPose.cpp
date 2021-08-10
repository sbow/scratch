// Shaun Bowman
// Aug 6 2021
// First program for Intel REalsense T265
// Recycle of example "pose" in librealsense examples

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
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

    // GET 10th frame, then continue debugging
    auto frames = pipe.wait_for_frames();
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
    for (int cnt = 0; cnt < 9; cnt++){
        // Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        pose_data = f.as<rs2::pose_frame>().get_pose_data();
    }
        
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


    // Output data to standard out
    std::cout << "\r" << "Device rotation w: " << std::setprecision(3) << std::fixed << qW << "\n";
    std::cout << "\r" << "Device rotation x: " << std::setprecision(3) << std::fixed << qX << "\n";
    return EXIT_SUCCESS;

} // END int main()

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
