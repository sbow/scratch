
#include <ros/ros.h>
#include <sstream>

// Global var to be populated by params.yaml
float r_driveratio = 0.0;
float k_spinloss = 0.0;
float a_motor_current_max = 0.0;
float a_motor_current_min = 0.0;
float kv_motor = 0.0;
int enum_AV_THROT_NC;
int enum_AV_THROT_INC;
int enum_AV_THROT_DEC;
int enum_AV_THROT_MAN;

// Global raw values from input topics
bool b_humanInLoop = false; // true if human holding dedman switch, else 0 throttle
float k_throtReqTele = 0.0; // unitless 0..1 throttle request from gamepad (teleop) mode
float k_throtReqAvTele = 0.0; // unitless 0..1 throttle override from gamepad in AV mode 
float M_axleTrqReqAv = 0.0; // mode: AV steering & throttle
float erpm_motor = 0.0; // erpm of motor

// Global state variables
const float EPSILON = 0.001; // for float equality test
float M_axleTrqReqTele = 0.0; // mode: tele (gamepad) steering & throttle
float M_axleTrqReqAvTele = 0.0; // mode: AV steering, manual throttle
float M_axleTrqReqAv = 0.0; // mode: full AV mode
float M_axleTrqCmd = 0.0; // arbitrated axle torque commanded
float i_motorCurrCmd = 0.0; // arbitrated motor current (ie: torq) commanded
float rpm_motor = 0.0; // rpm of motor
float w_motor = 0.0; // rad/s of motor

bool floatIsEq(float x, float y)
{
    bool isEqual = false;
    if (std::abs(x - y) < EPSILON)
        isEqual = true;
    return isEqual;
}

bool zeroOutStateVars()
{
    // Purpose:
    // Force to zero all variables on motor torque path
    M_axleTrqReqTele = 0.0;
    M_axleTrqReqAvTele = 0.0;
    M_axleTrqReqAv = 0.0;
    M_axleTrqCmd = 0.0;
    i_motorCurrCmd = 0.0;
    rpm_motor = 0.0;
    w_motor = 0.0;

}

void humanInLoopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of human-in-the-loop (ie: holding ready
    // button on the gamepad) topic 
    b_humanInLoop = msg->data;
}

void teleThrotCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of teleop (gamepad) throttle (0..1) command
    k_throtReqTele = msg->data;
}

void teleAvThrtCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of autonomous vehicle axle torque request
    k_throtReqAvTele = msg->data;
}

void avAxleTrqReqCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of AV control axle torque request
    M_axleTrqReqAv = msg->data;
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "axl_trq_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Get input parameters - motor, rover ect
    bool gotParam = true;
    gotParam = gotParam && n.getParam("r_driveratio",r_driveratio);
    gotParam = gotParam && n.getParam("k_spinloss",k_spinloss);
    gotParam = gotParam && n.getParam("a_motor_current_max",a_motor_current_max);
    gotParam = gotParam && n.getParam("a_motor_current_min",a_motor_current_min);
    gotParam = gotParam && n.getParam("kv_motor",kv_motor);
    gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_NC",enum_AV_THROT_NC);
    gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_INC",enum_AV_THROT_INC);
    gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_DEC",enum_AV_THROT_DEC);
    gotParam = gotParam && n.getParam("/teleop_logi/AV_THROT_MAN",enum_AV_THROT_MAN);
    if (not gotParam)
        ROS_FATAL("Failed to get parameters for axl_trq_arb node - motor teli ect");

    // Subscribe to input node topic's - teleop, av
    ros::Subscriber sub_teleThrot = n.subscribe("/teleop_logi/tele_throt_req",1000,teleThrotCallback);
    ros::Subscriber sub_avAxleTrq = n.subscribe("/av_axletrq_req",1000,avAxleTrqReqCallback);
    ros::Subscriber sub_teleAvThrot = n.subscribe("/teleop_logi/av_throt_req",1000,teleAvThrotCallback);
    ros::Subscriber sub_humanInLoop = n.subscribe("/teleop_logi/human_in_loop",1000,humanInLoopCallback);
    // TODO: need motor RPM (not erpm)
    // TODO: why is vesc driver node not running?

    // Publish to output topic's - motor current command to vesc, torque cmd
    ros::Publisher pub_i_motorCurrCmd = n.advertise("/axle_arb/i_motorCurrCmd", 5);
    std_msgs::Float32 msg_i_motorCurrCmd;
    ros::Publisher pub_M_axleTrqCmd = n.advertise("/axle_arb/M_axleTrqCmd", 5);
    std_msgs::Float32 msg_M_axleTrqCmd;

    // Announce node is running
    ROS_INFO("axl_trq_arb node - RUNNING");

    while (ros::ok())
    {
        // Arbitrate requestors - prio order: dedman, tele, av-tele, av, default
        if(not b_humanInLoop)
        {
            // base case, no human holding dedman switch, neutral axle torque cmd
            i_motorCurrCmd = 0.0;
            M_axleTrqCmd = 0.0;
        }
        else if(not floatIsEq(0.0, M_axleTrqReqTele))
        {
            // tele (gamepad) axle torque joystick outputing non-default value, tele cntrl
            i_motorCurrCmd = // TODO: function process M_axleTrqReqTele into motor current;
            M_axleTrqCmd = ;
        }
        else if(not floatIsEq(0.0, M_axleTrqReqAvTele))
        {
            // autonomous (av) axle torque control override to manual teleop value (gamepad)
            i_motorCurrCmd = ;
            M_axleTrqCmd = ;
        }
        else if(not floatIsEq(0.0, M_axleTrqReqAv))
        {
            // autonomous (av) axle torque control 
            i_motorCurrCmd = ;
            M_axleTrqCmd = ;
        }
        else
        {
            // catch-all, output defaults
            zeroOutStateVars();
            i_motorCurrCmd = 0.0;
            M_axleTrqCmd = 0.0;
        }
        msg_i_motorCurrCmd.data = i_motorCurrCmd; // arbitrated servo duty command
        msg_M_axleTrqCmd.data = M_axleTrqCmd; // arbitrated road wheel angle commanded

        // Publish output nodes:
        pub_i_motorCurrCmd.publish(msg_i_motorCurrCmd);
        pub_M_axleTrqCmd.publish(msg_M_axleTrqCmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}