
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <sstream>

// Global vars to store button setting parameters
int tele_steer_ax;
int tele_throt_ax;
int tele_dedman_btn;
int av_dedman_btn;
int av_start_btn;
int av_pause_btn;
int av_throtup_btn;
int av_throtdn_btn;
int ros_start_btn;
int ros_stop_btn;

// Global vars to store button values
const int32_t BTN_ON = 1;
float tele_steer_flt = 0.0;
float tele_throt_flt = 0.0;
bool tele_dedman_b = false;
bool av_dedman_b = false;
bool av_start_b = false;
bool av_pause_b = false;
bool av_throtup_b = false;
bool av_throtdn_b = false;
bool ros_start_b = false;
bool ros_stop_b = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    std::vector<int32_t> button = msg->buttons;
    std::vector<float> axes = msg->axes;
    tele_dedman_b = button[tele_dedman_btn] == 1;
    av_dedman_b = button[av_dedman_btn] == 1;
    av_start_b = button[av_start_btn] == 1;
    av_pause_b = button[av_pause_btn] == 1;
    av_throtup_b = button[av_throtup_btn] == 1;
    av_throtdn_b = button[av_throtdn_btn] == 1;
    ros_start_b = button[ros_start_btn] == 1;
    ros_stop_b = button[ros_stop_btn] == 1;
    tele_steer_flt = axes[tele_steer_ax];
    tele_throt_ax = axes[tele_throt_ax];
    ROS_INFO("tele_dedman_btn val: %d", tele_dedman_b); // debug
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_logi");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

// Read in button assignments from params.yaml

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

// Subscribe to the ROS Driver node - joy - to get raw gamepad button presses
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);

// Publish teleop state for use in axl_trq_arb node
  ros::Publisher pub = n.advertise<std_msgs::Bool>("/teleop_logi/dedman_depressed",5);
  std_msgs::Bool dedman_depressed;

  while (ros::ok())
  {
    // process dedman switch
    dedman_depressed.data = tele_dedman_b || av_dedman_b;
    pub.publish(dedman_depressed);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}