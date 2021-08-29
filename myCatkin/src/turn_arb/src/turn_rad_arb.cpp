#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cmath> // for atan(x) returning range[-pi/2, pi/2]

// Global var from param
float l_wheelbase = 0.0;
float l_track = 0.10; // divide by 0 protection
float l_minturnradphys = 0.0;
float k_steer_ang_to_servo_gain = 0.0;
float k_steer_ang_to_servo_offset = 0.0;


// Global state vars
bool b_teleSteerArb = false; // true if tele-op (gamepad) in control of steering
bool b_avSteerArb = false; // true if AV in control of steering
float l_turnRadCmd = 1000;
float k_servoDutyCmd = 0.5;
float a_radianRwaCmd = 0.0; // radians [-pi/2, pi/2]
const float EPSILON = 0.001; // for float equality test

// Global raw values from input topics
bool b_humanInLoop = false;
float k_servoDutyReqTele = 0.0;
float l_turnRadReqAv = 0.0;

bool floatIsEq(float x, float y)
{
    bool isEqual = false;
    if (std::abs(x - y) < EPSILON)
        isEqual = true;
    return isEqual;
}
float radianRwaFromTurnRad(float turnRadCmd)
{
    // Function:
    // Return road-wheel-angle command based on arbitrating turn radius command
    // Basic formula:
    /*
    L == wheelbase
    R == turn rad commanded
    T == track width
    road wheel ang, front, inside = tan-1(L/(R-T/2))
    **/

   float turnRadArb = std::max(std::abs(turnRadCmd), std::abs(l_minturnradphys));
   float radiansDesireInsideRwa = atan(l_wheelbase/(turnRadArb - l_track/2.0)); 
   float radiansDesireOutsideRwa = atan(l_wheelbase/(turnRadArb + l_track/2.0)); 
   float radiansDesireArbRwa = (radiansDesireInsideRwa+radiansDesireOutsideRwa)/2.0;

   // Debug:
   ROS_INFO("turn rad request: %f", turnRadArb);
   ROS_INFO("rwa rad inside: %f", radiansDesireInsideRwa);
   ROS_INFO("rwa rad outside: %f", radiansDesireOutsideRwa);
   ROS_INFO("rwa rad arb: %f", radiansDesireArbRwa);

   return radiansDesireArbRwa;
}

float dutyFromAng(float angCmd)
{
    // Function:
    // Return servo dutry cycle to be commanded given desired road wheel angle
    // Basic formula:
    /*
    k1 == proportional constant
    k2 == offset constant
    duty = k1*angCmd+k2
    */

   float dutyCmd = k_steer_ang_to_servo_gain*angCmd + k_steer_ang_to_servo_offset;

   // Debug:
   ROS_INFO("servo duty cmd: %f", dutyCmd);

   return dutyCmd;
}

float angFromDuty(float dutyCmd)
{
    // Function:
    // Return angle from duty cycle. This is used to populate a topic when
    // in teleop steer mode (ie: joystick on gamepad). The resulting road
    // wheel angle commanded might be useful to kinematic state estimation.
    float angCmd = (dutyCmd - k_steer_ang_to_servo_offset)/k_steer_ang_to_servo_gain;
    
    // Debug:
    ROS_INFO("ang from duty cmd: %f", angCmd);

    return angCmd;
}

void teleSteerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Callback triggered upon publication of new AV steering command (duty raw)
    k_servoDutyReqTele = msg->data;
}

void avSteerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Callback triggered upon publication of AV steering command (turn radius desired)
    l_turnRadReqAv = msg->data;
}

void humanInLoopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of human-in-the-loop (ie: holding ready
    // button on the gamepad) topic 
    b_humanInLoop = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Read in parameters to global var's
    bool gotParam = true;
    gotParam = gotParam && n.getParam("l_wheelbase", l_wheelbase);
    gotParam = gotParam && n.getParam("l_track", l_track);
    gotParam = gotParam && n.getParam("l_minturnradphys", l_minturnradphys);
    gotParam = gotParam && n.getParam("k_steer_ang_to_servo_gain", k_steer_ang_to_servo_gain);
    gotParam = gotParam && n.getParam("k_steer_ang_to_servo_offset", k_steer_ang_to_servo_offset);
    if (not gotParam)
        ROS_FATAL("Failed to load l_wheelbase / track / min turn rad / servo parameters");

    // Subscribe to input topics (from teleop_logi, and AV controllers)
    ros::Subscriber sub_teleSteer = n.subscribe("/teleop_logi/tele_steer_req", 1000, teleSteerCallback);
    ros::Subscriber sub_avSteer = n.subscribe("av_steer_req", 1000, avSteerCallback);
    ros::Subscriber sub_humanInLoop = n.subscribe("teleop_logi/human_in_loop", 1000, humanInLoopCallback);

    // Publish output topics (Road wheel angle cmd, Servo duty cmd)
    ros::Publisher pub_rwaRadiansCmd = n.advertise<std_msgs::Float32>("/turn_arb/rwaRadiansCmd",5);
    std_msgs::Float32 rwaRadiansCmd;
    ros::Publisher pub_servoDutyCmd = n.advertise<std_msgs::Float32>("/turn_arb/servoDutyCmd",5);
    std_msgs::Float32 servoDutyCmd;

    // Announce node is running
    ROS_INFO("turn_rad_arb node - RUNNING");

    // Debug:
    //float bub = radiansFromTurnRad(3);
    //float glub = dutyFromAng(bub);

    while (ros::ok())
    {
        // Determine control condition: prioritize dedman, then tele, then av, then default
        if(not b_humanInLoop)
        {
            // base case, no human holding dedman switch, neutral steer cmd
            k_servoDutyCmd = 0.5;
            a_radianRwaCmd = 0.0;
        }
        else if(not floatIsEq(0.5, k_servoDutyReqTele))
        {
            // tele (gamepad) steer joystick outputing non-default value, tele cntrl
            k_servoDutyCmd = k_servoDutyReqTele;
            a_radianRwaCmd = angFromDuty(k_servoDutyCmd);
        }
        else if(not floatIsEq(0.0, l_turnRadReqAv))
        {
            // autonomous (av) steer control topic outputting real turn radius req
            a_radianRwaCmd = radianRwaFromTurnRad(l_turnRadReqAv);
            k_servoDutyCmd = dutyFromAng(a_radianRwaCmd);
        }
        else
        {
            // catch-all, output defaults
            k_servoDutyCmd = 0.5;
            a_radianRwaCmd = 0.0;
        }
        servoDutyCmd.data = k_servoDutyCmd; // arbitrated servo duty command
        rwaRadiansCmd.data = a_radianRwaCmd; // arbitrated road wheel angle commanded

        // Publish output nodes:
        pub_rwaRadiansCmd.publish(rwaRadiansCmd);
        pub_servoDutyCmd.publish(servoDutyCmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}