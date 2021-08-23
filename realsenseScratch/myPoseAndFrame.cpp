// Shaun Bowman
// 2021/08/12 - Friday
// Remake of intel realsense example rs-pose-and-image.cpp
// For eventual use in ROS for control of rover


// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <algorithm>
#include <vector>
#include <array>

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
    float myFloat[4] = {1.0,2.0,3.0,4.0};
    std::vector<float> myVec;
    std::vector<float> myEmptyVec;
    std::vector<float>::iterator myVecIt;
    myVecIt = myVec.begin();
    for (int ix = 0; ix < 4; ix++)
    {
        myVec.push_back(myFloat[ix]);
    }
    

    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE }, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // Define frame callback

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex data_mutex;
    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        // Only start measuring time elapsed once we have received the
        // first piece of data
        if (first_data) {
            first_data = false;
            last_print = std::chrono::system_clock::now();
        }

        if (auto fp = frame.as<rs2::pose_frame>()) {
            pose_counter++;
        }
        else if (auto fs = frame.as<rs2::frameset>()) {
            frame_counter++;
        }

        // Print the approximate pose and image rates once per second
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(0) << std::fixed
                      << "Pose rate: "  << pose_counter << " "
                      << "Image rate: " << frame_counter << std::flush;
            pose_counter = 0;
            frame_counter = 0;
            last_print = now;
        }
    };

    // Start streaming through the callback
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    // Sleep this thread until we are done
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
