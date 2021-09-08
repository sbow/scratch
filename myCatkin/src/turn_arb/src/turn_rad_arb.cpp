#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath> // for atan(x) returning range[-pi/2, pi/2]

#define T_LEFT 1
#define T_RIGHT 2
#define T_CENTER 3
#define N_SERVO_CHAN 16

// Global var from param
int k_servo_left = 3800.0;
int k_servo_center = 5100.0;
int k_servo_right = 6700.0;
int servo_channel = 0;
std::vector<float> table_pwmcmd_left_servo;
std::vector<float> table_pwmcmd_right_servo;
std::vector<float> axis_turnrad_servo;

// Global state vars
bool b_teleSteerArb = false; // true if tele-op (gamepad) in control of steering
bool b_avSteerArb = false; // true if AV in control of steering
float l_turnRadCmd = 1000;
int k_servoDutyCmd = k_servo_center;
const float EPSILON = 0.001; // for float equality test
int enum_steerDir = T_CENTER; //
float k_servoLinScale_scaleL = 0;
float k_servoLinScale_offstL = 0;
float k_servoLinScale_scaleR = 0;
float k_servoLinScale_offstR = 0;

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

std::vector<int> getNeighborsIx(std::vector<float> axisInc, float axisSearch)
{
    // Purpose:
    // Given monotonically increasing axis - axisInc, and axis value of interest axisSearch,
    // return the indicies of the numerically neigboring axis values.
    // The first value in the returned vector gives the number of neighbors.
    // ie: axisInc = {1, 2, 3, 4}; axisSize = 4, axisVal=3.5; neighbor vals: 3,4 returns: {2,2,3}
    // ie2:axisInc = {1, 2, 3, 4}; axisSize = 4, axisVal=-1; neighbor vals: 1 returns: {1,0}
    // Note:
    // in Cpp, array as parameter decay's to a pointer to first element


    int ixTest = 0;
    float axisVal = axisInc[ixTest];
    int nNeighbor = 1; // for point in list, at least 1 neighbor, max of 2
    int axisSize = axisInc.size();
    std::vector<int> nAndNeighborsIx = {nNeighbor,0,0};
    ixTest = 1;

    if (axisSearch > axisVal && not floatIsEq(axisSearch, axisVal))
    {
        // continue if not special case where
        //  axisSearch is "Left" numerically of first element
        while (ixTest < axisSize)
        {
            axisVal = axisInc[ixTest];
            if (axisSearch < axisVal)
            {
                // stop search, element left of ixTest is neighbor
                nNeighbor = 2;
                nAndNeighborsIx[0] = nNeighbor; // first element is n neibors
                nAndNeighborsIx[1] = ixTest - 1; // index of first val lt axisSearch
                nAndNeighborsIx[2] = ixTest; // index of first val gt axisSearch
                break;
            }
            ixTest += 1;
        }
        if (nNeighbor == 1)
        {
            // other special case, axisSearch is "Right" of right-most array val
            nAndNeighborsIx[1] = axisSize - 1;
        }
    }
    return nAndNeighborsIx;
}

float floatInterp1(std::vector<float> xaxis, std::vector<float> table, float xval)
{
    // Purpose:
    // 1D lookup table
    // endpoint and beyond: latch value
    // algo:
    // determine neighboring axis values of input xval yval
    // directly lookup the four values from the table corressponding to the neighbors
    // determine normalized position of xval yval to neighboring axis values
    // multiply 
    // imp notes:
    //     _|___
    //    x1|a1
    //    x2|a2   <---- table
    //    ..|    
    //    xm|am
    //    ^xaxis

    float interp1Result = 0.0; // return value stored here
    // std::vector<float> xaxis_f(xaxis.begin(), xaxis.end());
    std::vector<int> nAndNeigX = getNeighborsIx(xaxis, xval);

    if (nAndNeigX[0] == 2)
    {
        // Normal case
        int x_neiL = xaxis[nAndNeigX[1]];
        int x_neiR = xaxis[nAndNeigX[2]];
        int y_neiL = table[nAndNeigX[1]];
        int y_neiR = table[nAndNeigX[2]];
        float normDist = (xval - x_neiL)/(x_neiR-x_neiL); // normalized relative position of xval to neighb axis pnts
        interp1Result = y_neiL + normDist*(y_neiR-y_neiL); // linear approximation
    }
    else if(nAndNeigX[0] == 1)
    {
        // Simple case, latch endpoint & return
        interp1Result = (float)table[nAndNeigX[1]];
    }
    else
    {
        // Unknown case lol
        // Shouldn't happen
        interp1Result = (float) k_servo_center;
    }

    return interp1Result; 
}

//      // NOTE:
//      // NOT USED....
//      //    instead an experimentally determined lookup table from turn rad to servo
//      //    command is implemented. The advantages are capturing non linearities in 
//      //    steering, and also basic divergence from ackermann steering geometry. 
//      //    Additionally, future expansion could capture impact of accel / vel on
//      //    effective turn radius.
//  float radianRwaFromTurnRad(float turnRadCmd)
//  {
//      // Function:
//      // Return road-wheel-angle command based on arbitrating turn radius command
//      // Basic formula:
//      /*
//      L == wheelbase
//      R == turn rad commanded
//      T == track width
//      road wheel ang, front, inside = tan-1(L/(R-T/2))
//      **/
//  
//     float turnRadArb = std::max(std::abs(turnRadCmd), std::abs(l_minturnradphys));
//     float radiansDesireInsideRwa = atan(l_wheelbase/(turnRadArb - l_track/2.0)); 
//     float radiansDesireOutsideRwa = atan(l_wheelbase/(turnRadArb + l_track/2.0)); 
//     float radiansDesireArbRwa = (radiansDesireInsideRwa+radiansDesireOutsideRwa)/2.0;
//  
//     // Debug:
//     ROS_INFO("turn rad request: %f", turnRadArb);
//     ROS_INFO("rwa rad inside: %f", radiansDesireInsideRwa);
//     ROS_INFO("rwa rad outside: %f", radiansDesireOutsideRwa);
//     ROS_INFO("rwa rad arb: %f", radiansDesireArbRwa);
//  
//     return radiansDesireArbRwa;
//  }

float servoCmdFromLinScale(float k_inputNorm)
{
    // Function:
    // In teleop (gamepad) operation, throttle position is range -1..1
    // This function makes use of exeperimentally derived servo pwm command
    // corresponding to full left, full right, center & scales the range linearly.

    float servoCmd = k_inputNorm;
    if (k_inputNorm > 0) // if LEFT
        servoCmd = k_servoLinScale_scaleL*k_inputNorm + k_servoLinScale_offstL;
    else if (k_inputNorm < 0) // if RIGHT, defined by joy node output / tele node
        servoCmd = k_servoLinScale_scaleR*k_inputNorm + k_servoLinScale_offstR;
    else
        servoCmd = k_servo_center;
    ROS_INFO("k_inputNorm: %f", k_inputNorm);
    ROS_INFO("k_servoLinScaleL: %f", k_servoLinScale_scaleL);
    ROS_INFO("k_servoLinScaleOffL: %f", k_servoLinScale_offstL);
    ROS_INFO("servoCmd: %f", servoCmd); 
    return servoCmd;
}

void teleSteerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Callback triggered upon publication of new AV steering command (duty raw)
    k_servoDutyReqTele = msg->data;
}

void avTurnRadCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Callback triggered upon publication of AV steering command (turn radius desired)
    l_turnRadReqAv = msg->data;
    if (l_turnRadReqAv > 0)
        enum_steerDir = T_LEFT;
    else if (l_turnRadReqAv < 0)
        enum_steerDir = T_RIGHT;
    else
        enum_steerDir = T_CENTER;
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
    gotParam = gotParam && n.getParam("servo_left", k_servo_left);
    gotParam = gotParam && n.getParam("servo_center", k_servo_center);
    gotParam = gotParam && n.getParam("servo_right", k_servo_right);
    gotParam = gotParam && n.getParam("k_pwmcmd_left_table_servo", table_pwmcmd_left_servo);
    gotParam = gotParam && n.getParam("k_pwmcmd_right_table_servo", table_pwmcmd_right_servo);
    gotParam = gotParam && n.getParam("m_turnrad_axis_servo", axis_turnrad_servo);
    gotParam = gotParam && n.getParam("servo_channel", servo_channel);
    if (not gotParam)
        ROS_FATAL("Failed to load l_wheelbase / track / min turn rad / servo parameters");

    // Subscribe to input topics (from teleop_logi, and AV controllers)
    ros::Subscriber sub_teleSteer = n.subscribe("/teleop_logi/tele_steer_req", 1000, teleSteerCallback);
    ros::Subscriber sub_avTurnRad = n.subscribe("av_turnrad_req", 1000, avTurnRadCallback);
    ros::Subscriber sub_humanInLoop = n.subscribe("teleop_logi/human_in_loop", 1000, humanInLoopCallback);

    // Publish output topics (Servo duty cmd)
    // servo command vector topic - subscribed by pca9685 node - 16 element int array correspond to pwm duty
    ros::Publisher pub_servoCmdVector = n.advertise<std_msgs::Int32MultiArray>("/servo/cmdVector",5);
    std_msgs::Int32MultiArray msg_servoCmdVector;
    msg_servoCmdVector.layout.dim.push_back(std_msgs::MultiArrayDimension());
    std::vector<int> servoCmdVectorData(N_SERVO_CHAN, -1);
    msg_servoCmdVector.layout.dim[0].size = servoCmdVectorData.size();
    msg_servoCmdVector.layout.dim[0].stride = 1;
    msg_servoCmdVector.layout.dim[0].label = "pwm_channel";

    // Announce node is running
    ROS_INFO("turn_rad_arb node - RUNNING");

    // Pre-process parameters to setup linear scale for servo command
    // At this time, this is used in teleop (gamepad) mode
    k_servoLinScale_offstL = k_servo_center; // "safe" for 0 input
    k_servoLinScale_scaleL = k_servo_left-k_servo_center; // neg scale, pos depend var
    k_servoLinScale_offstR = k_servo_center; // "safe" for 0 input
    k_servoLinScale_scaleR = k_servo_center-k_servo_right; // neg scale, neg depend var


    while (ros::ok())
    {
        // Determine control condition: prioritize dedman, then tele, then av, then default
        if(not b_humanInLoop)
        {
            // base case, no human holding dedman switch, neutral steer cmd
            k_servoDutyCmd = k_servo_center;
        }
        else if(not floatIsEq(0.5, k_servoDutyReqTele))
        {
            // tele (gamepad) steer joystick outputing non-default value, tele cntrl
            k_servoDutyCmd = (int)std::round(servoCmdFromLinScale(k_servoDutyReqTele));
        }
        else if(not floatIsEq(0.0, l_turnRadReqAv))
        {
            // autonomous (av) steer control topic outputting real turn radius req
            if (l_turnRadReqAv > 0)
            {
                // LEFT positive in Mowbot coordinate system
                k_servoDutyCmd = floatInterp1(axis_turnrad_servo, table_pwmcmd_left_servo, l_turnRadReqAv);
            }
            else if (l_turnRadReqAv < 0)
            {
                // RIGHT negative in Mowbot coordinate system
                k_servoDutyCmd = floatInterp1(axis_turnrad_servo, table_pwmcmd_right_servo, l_turnRadReqAv);
            }
        }
        else
        {
            // catch-all, output defaults
            k_servoDutyCmd = k_servo_center;
        }
        // Build servo command message - 16 element int vector corresponding to pwm channel
        msg_servoCmdVector.data.clear();
        msg_servoCmdVector.data.insert( msg_servoCmdVector.data.end(), 
                                        servoCmdVectorData.begin(),
                                        servoCmdVectorData.end());
        k_servoDutyCmd = std::round(k_servoDutyCmd); // round arbitrated command for max accuracy
        msg_servoCmdVector.data[servo_channel] = (int)k_servoDutyCmd; // populate steering servo channel

        // Publish output nodes:
        pub_servoCmdVector.publish(msg_servoCmdVector);

        // loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}