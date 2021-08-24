
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <vector>
#include <sstream>

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::vector<int32_t> button = msg->buttons;
    std::vector<float> axes = msg->axes;
    ROS_INFO("heard something");
    //ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_logi");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

// Read in button assignments from params.yaml
  int tele_dedman_btn;
  int tele_steer_ax;
  int tele_throt_ax;
  int av_dedman_btn;
  int av_start_btn;
  int av_pause_btn;
  int av_throtup_btn;
  int av_throtdn_btn;
  int ros_start_btn;
  int ros_stop_btn;

  bool gotParam = true; 

  gotParam = n.getParam("/teleop_logi/tele_dedman_btn", tele_dedman_btn); 
  gotParam = n.getParam("/teleop_logi/tele_steer_ax", tele_steer_ax); 
  gotParam = n.getParam("/teleop_logi/tele_throt_ax", tele_throt_ax); 
  gotParam = n.getParam("/teleop_logi/av_dedman_btn", av_dedman_btn); 
  gotParam = n.getParam("/teleop_logi/av_start_btn", av_start_btn); 
  gotParam = n.getParam("/teleop_logi/av_pause_btn", av_pause_btn); 
  gotParam = n.getParam("/teleop_logi/av_throtup_btn", av_throtup_btn); 
  gotParam = n.getParam("/teleop_logi/av_throtdn_btn", av_throtdn_btn); 
  gotParam = n.getParam("/teleop_logi/ros_start_btn", ros_start_btn); 
  gotParam = n.getParam("/teleop_logi/ros_stop_btn", ros_stop_btn); 
  
  if(not gotParam)
  {
      ROS_FATAL("Didnt get tele_xxxx_yyy param");
  };

  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);



  while (ros::ok())
  {
    ROS_INFO("tele_dedman_btn: %d", tele_dedman_btn);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}