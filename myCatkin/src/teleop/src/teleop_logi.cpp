
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
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
int enum_AV_THROT_NC;
int enum_AV_THROT_INC;
int enum_AV_THROT_DEC;

// Global state vars

// Set global vars upon publication of new Joy topic
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
    tele_throt_flt = axes[tele_throt_ax];
    ROS_INFO("tele_dedman_btn val: %d", tele_dedman_b); // debug
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_logi");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  // Read in button assignments from params.yaml

  bool gotParam = true; 

  gotParam = gotParam && n.getParam("/teleop_logi/tele_dedman_btn", tele_dedman_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/tele_steer_ax", tele_steer_ax); 
  gotParam = gotParam && n.getParam("/teleop_logi/tele_throt_ax", tele_throt_ax); 
  gotParam = gotParam && n.getParam("/teleop_logi/av_dedman_btn", av_dedman_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/av_start_btn", av_start_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/av_pause_btn", av_pause_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/av_throtup_btn", av_throtup_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/av_throtdn_btn", av_throtdn_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/ros_start_btn", ros_start_btn); 
  gotParam = gotParam && n.getParam("/teleop_logi/ros_stop_btn", ros_stop_btn); 
  
  if(not gotParam)
  {
      ROS_FATAL("Didnt get tele_xxxx_yyy param");
  };

  // Subscribe to the ROS Driver node - joy - to get raw gamepad button presses
  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);

  // Publish teleop state for use in axl_trq_arb node
  ros::Publisher pub_hil = n.advertise<std_msgs::Bool>("/teleop_logi/human_in_loop",5);
  std_msgs::Bool human_in_loop;
  gotParam = true; // reset gotParam tracking boolean
  gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_NC",enum_AV_THROT_NC);
  gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_INC",enum_AV_THROT_INC);
  gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_DEC",enum_AV_THROT_DEC);
  if(not gotParam)
    ROS_FATAL("Didn't get AV_THROT_XX enum param");

  // Publish AV throttle request 
  ros::Publisher pub_avthrot = n.advertise<std_msgs::Int8>("/teleop_logi/av_throt_req",5);
  std_msgs::Int8 av_throt_req;

  // Publish teleop throttle and steering
  ros::Publisher pub_telesteer = n.advertise<std_msgs::Float32>("/teleop_logi/tele_steer_req",5);
  std_msgs::Float32 tele_steer_req;
  ros::Publisher pub_telethrot = n.advertise<std_msgs::Float32>("/teleop_logi/tele_throt_req",5);
  std_msgs::Float32 tele_throt_req;

  // Publish ROS Bag record request

  while (ros::ok())
  {
    // process dedman switch
    human_in_loop.data = tele_dedman_b || av_dedman_b;
    pub_hil.publish(human_in_loop);

    // process AV throttle request
    if(av_throtup_b && av_dedman_b)
        av_throt_req.data = enum_AV_THROT_INC;
    else if(av_throtdn_b && av_dedman_b)
        av_throt_req.data = enum_AV_THROT_DEC;
    else
        av_throt_req.data = enum_AV_THROT_NC;
    pub_avthrot.publish(av_throt_req);

    // process teleop throttle and steering
    if(tele_dedman_b)
    {
        tele_steer_req.data = tele_steer_flt;
        tele_throt_req.data = tele_throt_flt;
    }
    else
    {
        tele_steer_req.data = 0.0;
        tele_throt_req.data = 0.5;
    }
    pub_telesteer.publish(tele_steer_req); // note: raw values 0..1; must be converted to
    pub_telethrot.publish(tele_throt_req); // physical units in axl_trq_arb

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}