
#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <cmath> // for std::floor ect
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

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
std::vector<float> xaxis_PedalMap_PctThrotTele;
std::vector<float> yaxis_PedalMap_RpmMotor;
std::vector<float> table_PedalMap_1d;


// Global raw values from input topics
bool b_humanInLoop = false; // true if human holding dedman switch, else 0 throttle
float k_throtReqTele = 0.0; // unitless 0..1 throttle request from gamepad (teleop) mode
int8_t enum_throtReqAvTele = 0.0; // unitless 0..1 throttle override from gamepad in AV mode 
float erpm_motor = 0.0; // erpm of motor

// Global state variables
const float EPSILON = 0.001; // for float equality test
float M_axleTrqReqTele = 0.0; // mode: tele (gamepad) steering & throttle
float M_axleTrqReqAvTele = 0.0; // mode: AV steering, manual throttle
float M_axleTrqReqAv = 0.0; // mode: full AV mode
float M_axleTrqCmd = 0.0; // arbitrated axle torque commanded
float M_axleTrqSpinloss = 0.0; // estimated current spin loss in units of axl torq
float M_motorTorqCmd = 0.0; // desired motor torque (actually command current)
float i_motorCurrCmd = 0.0; // arbitrated motor current (ie: torq) commanded
float rpm_motor_raw = 0.0; // rpm of motor
float rpm_motor = 0.0; // rpm of motor, filtered
float w_motor = 0.0; // rad/s of motor

// Utillity functions
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
    M_axleTrqSpinloss = 0.0;
    M_motorTorqCmd = 0.0;
    i_motorCurrCmd = 0.0;
    rpm_motor_raw = 0.0;
    rpm_motor = 0.0;
    w_motor = 0.0;
    return true;
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

float getRelDistToLNorm(float lVal, float rVal, float testVal)
{
    // Purpose:
    // Return relative distance of testVal to lVal and rVal, normalized to 0..1
    // NOTE: Unused
    float relDistNormL = (testVal - lVal)/(rVal - lVal);
    return relDistNormL;
}

float floatInterp2(std::vector<float> xaxis, std::vector<float> yaxis, std::vector<std::vector<float>> table, float xval, float yval)
{
    // Purpose:
    // 2D lookup table
    // endpoint and beyond: latch value
    // algo:
    // determine neighboring axis values of input xval yval
    // directly lookup the four values from the table corressponding to the neighbors
    // determine normalized position of xval yval to neighboring axis values
    // multiply 
    // imp notes:
    //     _|y0|y1|y2|..|yn   <--yaxis
    //    x1|a1|b1|c1|..|   <-:
    //    x2|a2|b2|c2|      <---- table 
    //    ..|
    //    xm|
    //    ^xaxis

    float interp2Result = 0.0; // return value to be stored here

    // get array size, assume table is size: [xSize,ySize]
    // UPDATE: done with template & pass by reference

    // Find indicies of axis values neighboring value of interest
    // returns: {nNeighbors 1||2, ixL, ixR}
    std::vector<int> nAndNeigX = getNeighborsIx(xaxis, xval);
    std::vector<int> nAndNeigY = getNeighborsIx(yaxis, yval);

    if(nAndNeigX[0] == 2 && nAndNeigY[0] ==2)
    {
        // Use weighted mean techniqe to compute bilinear interpolation
        // Normal case - xval & yval are within limits of xaxis and yaxis
        float denom = (xaxis[nAndNeigX[2]]-xaxis[nAndNeigX[1]])*
                      (yaxis[nAndNeigY[2]]-yaxis[nAndNeigY[1]]);
        float w11 = (xaxis[nAndNeigX[2]] - xval)*(yaxis[nAndNeigY[2]] - yval)/denom;
        float w12 = (xaxis[nAndNeigX[2]] - xval)*(yval - yaxis[nAndNeigY[1]])/denom;
        float w21 = (xval - xaxis[nAndNeigX[1]])*(yaxis[nAndNeigY[2]] - yval)/denom;
        float w22 = (xval - xaxis[nAndNeigX[1]])*(yval - yaxis[nAndNeigY[1]])/denom;
        interp2Result = w11*table[nAndNeigX[1]][nAndNeigY[1]] +
                        w12*table[nAndNeigX[1]][nAndNeigY[2]] +
                        w21*table[nAndNeigX[2]][nAndNeigY[1]] +
                        w22*table[nAndNeigX[2]][nAndNeigY[2]];
    }
    else if(nAndNeigX[0] == 1 && nAndNeigY[0] == 1)
    {
        // Simple case, value is outside both axis - return most extreme point
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]];
    }
    else if(nAndNeigX[0] == 1)
    {
        // Case xval is outside of xaxis, yval is within y axis
        float distToBound = getRelDistToLNorm(yaxis[nAndNeigY[1]], yaxis[nAndNeigY[2]], yval);
        float delta =   table[nAndNeigX[1]][nAndNeigY[2]] -
                        table[nAndNeigX[1]][nAndNeigY[1]];
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]] + 
                        delta*distToBound;
    }
    else if(nAndNeigY[0] == 1)
    {
        // Case yval is outside of yaxis, xval is within x axis
        float distToBound = getRelDistToLNorm(xaxis[nAndNeigX[1]], xaxis[nAndNeigX[2]], xval);
        float delta =   table[nAndNeigX[2]][nAndNeigY[1]] -
                        table[nAndNeigX[1]][nAndNeigY[1]];
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]] + 
                        delta*distToBound;
    }
    else
    {
        // Catch-all, shouldn't happen
        interp2Result = 0.0042069; // yolo 
    }

    return interp2Result;
}

// Physics functions
float getSpinLossAxlTorq(float motorRpm)
{
    // Returns: estimated lost axle torque due to driveline friction losses (ie: proportional to RPM)
    // Uses:
    // Global motor speed (rpm), note: this should be filtered to avoid oscilation in command
    return motorRpm*k_spinloss;
}
float getMotorTorqFromAxlTorq(float axlTorqDes)
{
    // Uses:
    // Tm = Tm,axle/driveRatio
    return axlTorqDes/r_driveratio;
}
float getMotorCurFromMotorTorq(float motorTorqDes)
{
    // Uses:
    // T~= 8.3*Ia/Kv 
    float Ia = 0.0;
    Ia = motorTorqDes*kv_motor/8.3f; // exp derivation: https://things-in-motion.blogspot.com/2018/12/how-to-estimate-torque-of-bldc-pmsm.html
    return Ia;
}


// Callback functions for input ROS topic variable data
void humanInLoopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Callback triggered upon publication of human-in-the-loop (ie: holding ready
    // button on the gamepad) topic 
    b_humanInLoop = msg->data;
}

void teleThrotCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Callback triggered upon publication of teleop (gamepad) throttle (0..1) command
    k_throtReqTele = msg->data;
}

void teleAvThrotCallback(const std_msgs::Int8::ConstPtr& msg)
{
    // Callback triggered upon publication of autonomous vehicle axle torque request
    // Form is Int8 encoded enum: 0==NC, 1==INC, 2==DEC, 3==MANUAL{gamepad}
    enum_throtReqAvTele = msg->data;
}

void avAxleTrqReqCallback(const std_msgs::Float32::ConstPtr& msg)
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
    gotParam = gotParam && n.getParam("rpm_motor_axis_telethrotmap",yaxis_PedalMap_RpmMotor);
    gotParam = gotParam && n.getParam("pct_throt_axis_telethrotmap",xaxis_PedalMap_PctThrotTele);
    gotParam = gotParam && n.getParam("M_axle_lookup_telethrotmap",table_PedalMap_1d);
    if (not gotParam)
        ROS_FATAL("Failed to get parameters for axl_trq_arb node - motor teli ect");

    // Reshape table_pedalMap_1d - from params.yaml - into 2D array of type std::vector<float>
    int nRow = xaxis_PedalMap_PctThrotTele.size();
    int nCol = yaxis_PedalMap_RpmMotor.size();
    int ixSource = 0;
    std::vector<float> colVals(nCol, 0);
    std::vector<std::vector<float>> table_PedalMap(nRow,colVals);   // 2D table of axle torque, reformed from 1D params.yaml vector. 
                                                                    // XrowLookup:Throt0..1 YrowLookup:MotorRpm
    for (int ixR = 0; ixR < nRow; ixR++)
    {
        for (int ixC = 0; ixC < nCol; ixC++)
        {
            table_PedalMap[ixR][ixC] = table_PedalMap_1d[ixSource];
            ixSource += 1;
        }
    }

    // Subscribe to input node topic's - teleop, av
    ros::Subscriber sub_teleThrot = n.subscribe("/teleop_logi/tele_throt_req",1000,teleThrotCallback);
    ros::Subscriber sub_avAxleTrq = n.subscribe("/av_axletrq_req",1000,avAxleTrqReqCallback);
    ros::Subscriber sub_teleAvThrot = n.subscribe("/teleop_logi/av_throt_req",1000,teleAvThrotCallback);
    ros::Subscriber sub_humanInLoop = n.subscribe("/teleop_logi/human_in_loop",1000,humanInLoopCallback);
    // TODO: need motor RPM (not erpm)
    // TODO: why is vesc driver node not running?

    // Publish to output topic's - motor current command to vesc, torque cmd
    // /commands/motor/current
    // Type: std_msgs/Float64
    //ros::Publisher pub_i_motorCurrCmd = n.advertise<std_msgs::Float32>("/axle_arb/i_motorCurrCmd", 5);
    ros::Publisher pub_i_motorCurrCmd = n.advertise<std_msgs::Float64>("/commands/motor/current", 5);
    std_msgs::Float64 msg_i_motorCurrCmd;
    ros::Publisher pub_M_axleTrqCmd = n.advertise<std_msgs::Float32>("/axle_arb/M_axleTrqCmd", 5);
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
        else if(not floatIsEq(0.0, k_throtReqTele))
        {
            // tele (gamepad) axle torque joystick outputing non-default value, tele cntrl
            // OR av steering & manual tele axle torque
            M_axleTrqReqTele = floatInterp2(xaxis_PedalMap_PctThrotTele, 
                                                yaxis_PedalMap_RpmMotor,
                                                table_PedalMap,
                                                k_throtReqTele,
                                                0.0f); //Note: 0 rpm override for now...
            M_axleTrqSpinloss = getSpinLossAxlTorq(100.0f); // 100 rpmx0.001=.1Nm axl trq test
            M_axleTrqCmd = M_axleTrqReqTele + M_axleTrqSpinloss;
            M_motorTorqCmd = getMotorTorqFromAxlTorq(M_axleTrqCmd);
            i_motorCurrCmd = getMotorCurFromMotorTorq(M_motorTorqCmd);
            M_axleTrqCmd = M_axleTrqReqTele;
        }
        else if(not floatIsEq(0.0, M_axleTrqReqAvTele))
        {
            // autonomous (av) axle torque control override to manual teleop value (gamepad)
            M_axleTrqSpinloss = getSpinLossAxlTorq(rpm_motor);
            M_axleTrqCmd = M_axleTrqReqAvTele + M_axleTrqSpinloss;
            M_motorTorqCmd = getMotorTorqFromAxlTorq(M_axleTrqCmd);
            i_motorCurrCmd = getMotorCurFromMotorTorq(M_motorTorqCmd);
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