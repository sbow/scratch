// Shaun Bowman
// 2021/10/15
// v0.0
// node name: av_pointcloud_node
// type: av_pointcloud

// * Purpose:
// 	* Drive automously mowbot through series of lat/lon points in sequence
// 	* Global 0 taken to be mowbot heading/position at start of controller action
// 	* Lookahead point, ie: kinematic path end point for planning purposes
// 	* Trajectory calculation
// 	* Conversion to turn radius desired
// 	* Command to turn_rad_arb
// 		*** av_turnrad_req** topic
// * Future:
// 	* Add throttle. At first, manual throttle
// 		* Slow for turns
// 	* Closed loop trajectory

/*
Series of lat lon pro
    x & y points
? Ability to reset Pose from Realsense?
--> perhaps create a rationalization of pose from RS

Calculate path length ?
Try to 

Idea's to try and generate drive path:
- RDP Algorithm to simplify path between points
- bezier curves?

*/

#define _USE_MATH_DEFINES
#define true 1
#define false 0
#define BE_VERBOSE 1

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <cmath>
#include <vector>
#include <lite/array.hpp> // lite_array : header only matrix algebra library

float A_MAX_ANG;
float A_MIN_ANG;
float A_MAX_LIN;
float A_MIN_LIN;
float V_MAX_ANG;
float V_MIN_ANG;
float V_MAX_LIN;
float V_MIN_LIN;
float RES_V_LIN;
float RES_V_ANG;
float RADIUS_OBS;
float DT_CNTRL;
float DT_EVAL;
float WEIGHT_HEAD;
float WEIGHT_DIST;
float WEIGHT_VELO;
float GOAL_X;
float GOAL_Y;
float RAD_GOAL_MET;

// global vars set during initialization
float V_INI_LIN;
float V_INI_ANG;
float T_INI;
float X_INI;
float Y_INI;
float N_SIM;
typedef lite::array<float[100][2]> typeLiteArrayObs;
typeLiteArrayObs ObstaclesXY;
int ObstaclesXY_n;
typedef lite::array<float[5]> typeLiteArrayKini; // Kinimatic state: [x,y,theta,v_lin,v_ang]
typedef lite::array<float[2]> typeLiteArrayXy; // Cartesian coords: [x,y]
typedef lite::array<float[2]> typeLiteArrayVw; // Linear and angular velocity: [v_lin, v_ang]
typedef lite::array<float[4]> typeLiteArrayVwin; // Vel window [vMinLin,vMaxLin,vMinAng,vMaxAng]
typedef lite::array<float[5]> typeLiteArrayObjFunc; // ObjFunc inputs: [vLin,vAng,head,dist,vel]
float toRadian(float degree)
{
    float radian;
    radian = degree/180.0*M_PI;
    return radian;
}

float toDegree(float radian)
{
    float degree;
    degree = radian/M_PI*180.0;
}

void simSetup()
{
    /*
    set calibrations to values to match sim_doDwa2.m Octave script - for validation
    */
    A_MAX_ANG = toRadian(50.0f);   // rad/s/s, max achievable angular accel
    A_MIN_ANG = toRadian(-50.0f);  // rad/s/s, min achievable angular accel
    A_MAX_LIN = 0.2;            // m/s/s, max achievable linear accel
    A_MIN_LIN = -0.2;           // m/s/s, min achievable linear accel
    V_MAX_ANG = toRadian(20.0f);   // rad/s, max achievable angular vel
    V_MIN_ANG = toRadian(-20.0f);   // rad/s, min achievable angular vel
    V_MAX_LIN = 1;              // m/s, max achievable linear vel
    V_MIN_LIN = -1;             // m/s, max achievable linear vel
    RES_V_LIN = 0.01;           // m/s, control resolution, search space size
    RES_V_ANG = toRadian(1.0f);    // rad, control resolution, search space size
    RADIUS_OBS = 1.5;           // m, assume ObstaclesXY extends this radially
    DT_CNTRL = 0.1;             // s, step size for control ie 1/hz loop rate
    DT_EVAL = 3.0;              // s, model traj for cntrl option v,w this long
    WEIGHT_HEAD = 0.2;          // objective func weight, heading wrt GoalXY
    WEIGHT_DIST = 0.1;          // objective func weight, dist to ObstaclesXY
    WEIGHT_VELO = 0.2;          // objective func weight, linear velocity
    GOAL_X = 10;                // m, x coordinate of desired final position
    GOAL_Y = 10;                // m, y coordinate of desired final position
    RAD_GOAL_MET = 0.5;         //m, if norm of distance is nearer than this to goal, control sez success

    // 
    // Initial conditions for simulation
    V_INI_LIN = 0.0; // m/s, T_INIial velocity at start of path planning
    V_INI_ANG = toRadian(0.0); // rad/s/s, T_INIial rotaional velocity, CCW pos
    T_INI = toRadian(90.0); // rad, T_INIial heading, CCW pos
    X_INI = 0.0; // T_INIial x coord
    Y_INI = 0.0; // T_INIial y coord
    N_SIM = 5000; // number of steps in simulation

    ObstaclesXY(0,0) = 0.0f; 
    ObstaclesXY(1,0) = 4.0f; 
    ObstaclesXY(2,0) = 4.0f; 
    ObstaclesXY(3,0) = 5.0f; 
    ObstaclesXY(4,0) = 5.0f; 
    ObstaclesXY(5,0) = 5.0f; 
    ObstaclesXY(6,0) = 5.0f; 
    ObstaclesXY(7,0) = 8.0f; 
    ObstaclesXY(8,0) = 8.0f; 
    ObstaclesXY(9,0) = 7.0f; 
    ObstaclesXY(0,1) = 2.0f; 
    ObstaclesXY(1,1) = 2.0f; 
    ObstaclesXY(2,1) = 4.0f; 
    ObstaclesXY(3,1) = 4.0f; 
    ObstaclesXY(4,1) = 5.0f; 
    ObstaclesXY(5,1) = 6.0f; 
    ObstaclesXY(6,1) = 9.0f; 
    ObstaclesXY(7,1) = 8.0f; 
    ObstaclesXY(8,1) = 9.0f; 
    ObstaclesXY(9,1) = 9.0f; 
    ObstaclesXY_n = 10;
}


lite::array<float[5]> f_movexy(lite::array<float[5]> x, lite::array<float[2]> traj)
{
    // Motion Model
    // x: [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] SB: initial state of robot
    // traj: trajectory of the control option [x,y]

    // F*x:
    // x_ini
    // y_ini
    // yaw_ini
    // 0_??note: is 0 here, because taken directly from traj(1) aka vlin_cur ???
    // 0 = a_ini

    //     traj:
    //     vlin_cur
    //     vang_cur
 
    // B:
    // DT_CNTRL*cos(thet_cur)    ?
    // DT_CNTRL*sin(thet_cur)    ?
    // ?                   DT_CNTRL
    // 1                   ?
    // ?                   1
    //
    // B*traj:
    // vlin_cur*cos(thet_cur)*DT_CNTRL = x_delta = "x comp of vlin"
    // vlin_cur*sin(thet_cur)*DT_CNTRL = y_delta = "y comp of vlin"
    // vang_cur*DT_CNTRL = theta_delta = "delta theta by constant ang vel * DT_CNTRL"
    // vlin_cur = vlin_delta
    // vang_cur = vang_delta

    // KiniState:
    // X_CUR = x_ini + x_delta
    // Y_CUR = y_ini + y_delta
    // T_CUR = t_ini + t_delta
    // V_CUR_LIN = v_ini + v_delta = 0 + v_delta = v_delta = vlin_cur =
    // vlin_control : note, implicitly assumes the vehicle simply achieves
    // the angular acceleration and velocity of the control signal. There is
    // no actual model of vehicle velocity; linear or angular.
    // This is essentially just saying
    // that on the next control loop, the linear vel / accel equals the
    // control system's commands. It does however model positoin and
    // heading.
    // V_CUR_ANG = vang_delta = vang_cur = vang_control

    float x_ini = x(0);
    float y_ini = x(1);
    float thet_ini = x(2);
    float v_ini_lin = x(3);
    float v_ini_ang = x(4);
   
    float v_cur_lin = traj(0);
    float v_cur_ang = traj(1);
   
    float x_delta = v_cur_lin*cos(thet_ini)*DT_CNTRL;
    float y_delta = v_cur_lin*sin(thet_ini)*DT_CNTRL;
    float thet_delta = v_cur_ang*DT_CNTRL;
   
    float x_final = x_ini + x_delta;
    float y_final = y_ini + y_delta;
    float thet_final = thet_ini + thet_delta;
   
    float v_final_lin = v_cur_lin; // assume car reaches command at end
    float v_final_ang = v_cur_ang; // assume car reaches command at end
   
    typeLiteArrayKini KiniState = typeLiteArrayKini(    
                                                x_final,
                                                y_final,
                                                thet_final,
                                                v_final_lin,
                                                v_final_ang);
    return KiniState;
}

typeLiteArrayVw f_doDwa(typeLiteArrayKini KiniState, typeLiteArrayObs ObstaclesXY)
{
    //DWA - Function to calculate the lin & ang vel value by
   
    float v_select_lin = 0; // placeholder, control choice of v linear m/s
    float v_select_ang = 0; // placeholder, control choice of v angular rad/s
   
    //--------------------------------------------------------------------
    //Dynamic Window Creation:
    //
    //Vs : Velocity window - based on robot propulsion limits
    typeLiteArrayVwin Vs;
    //Vs= typeLiteArrayVwin(  V_MIN_LIN, 
    Vs= typeLiteArrayVwin(  0, 
                            V_MAX_LIN, 
                            V_MIN_ANG, 
                            V_MAX_ANG);
   
    if (BE_VERBOSE) std::cout << "\nvs:\n" << Vs;
    // Vd : Velocity window - ability to hit final velocity by next loop, ie: acl lims.
    // note: i believe this should be DT_EVAL (ie: ability to stop within
    // next PLANNING phase, not control loop... far too limiting?
    // KiniState = [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] initial state of robot
    float v_cur_lin = KiniState(3);
    float v_cur_ang = KiniState(4);
    typeLiteArrayVwin Vd; 
    float v_cur_linAbs = std::abs(v_cur_lin);
    Vd= typeLiteArrayVwin(  v_cur_lin+A_MIN_LIN*DT_CNTRL,  
                            v_cur_lin+A_MAX_LIN*DT_CNTRL,  
                            v_cur_ang+A_MIN_ANG*DT_CNTRL,  
                            v_cur_ang+A_MAX_ANG*DT_CNTRL); // note: should remove constraint when v_cur small
    if (BE_VERBOSE) std::cout << "\nvd:\n" << Vd;
    // Arb above to gen control space limits for v & w:
    float v_limmin_lin = std::max(Vs(0), Vd(0));
    float v_limmax_lin = std::min(Vs(1), Vd(1));
    float v_limmin_ang = std::max(Vs(2), Vd(2));
    float v_limmax_ang = std::min(Vs(3), Vd(3));
   
    if (BE_VERBOSE) std::cout << "\nv_limmin_lin:\n" << v_limmin_lin;
    if (BE_VERBOSE) std::cout << "\nv_limmax_lin:\n" << v_limmax_lin;
    if (BE_VERBOSE) std::cout << "\nv_limmin_ang:\n" << v_limmin_ang;
    if (BE_VERBOSE) std::cout << "\nv_limmax_ang:\n" << v_limmax_ang;
    //--------------------------------------------------------------------    
    // Loop through control options:
    //
    // For each option:
    // Loop for DT_EVAL time and compute expected final position / trajectory
    // Determine the heading angle between position AFTER move & goal.
    // Determine distance between final position and NEAREST object, note: this
    //     imposes a constraint, interim points are not checked for distance to
    //     obstacle. So if vel is large or DT_CNTRL is large, u might skip past a
    //     point where host was very close to obstacle.
    // Compute objective function using weighting params.
    // Store results of objective function, and trajectories, for later analysis
    float xi; // float to build up vectors;
    int ni;

    int n_vcntrl_lin = std::round((v_limmax_lin-v_limmin_lin)/RES_V_LIN)+1;
    lite::array<float[1]> V_CNTRL_LIN(n_vcntrl_lin);
    ni = 0;
    xi = v_limmin_lin;
    while (ni < n_vcntrl_lin)
    {
        V_CNTRL_LIN(ni) = xi;
        ni = ni + 1;
        xi = xi + RES_V_LIN;
    }
    if (BE_VERBOSE) std::cout << "\nn_vcntrl_lin:\n" << n_vcntrl_lin;
    if (BE_VERBOSE) std::cout <<"\nV_CNTRL_LIN:\n";
    if (BE_VERBOSE) std::cout << V_CNTRL_LIN;


    int n_vcntrl_ang = std::round((v_limmax_ang-v_limmin_ang)/RES_V_ANG)+1;
    lite::array<float[1]> V_CNTRL_ANG(n_vcntrl_ang);
    ni = 0;
    xi = v_limmin_ang;
    while (ni < n_vcntrl_ang)
    {
        V_CNTRL_ANG(ni) = xi;
        ni = ni + 1;
        xi = xi + RES_V_ANG;
    }
    if (BE_VERBOSE) std::cout << "\nn_vcntrl_ang:\n" << n_vcntrl_ang;
    if (BE_VERBOSE) std::cout <<"\nV_CNTRL_ANG:\n";
    if (BE_VERBOSE) std::cout << V_CNTRL_ANG;


    int n_time_cntrl = std::round((DT_EVAL - DT_CNTRL)/DT_CNTRL)+1;
    lite::array<float[1]> TIME_CNTRL(n_time_cntrl);
    ni = 0;
    xi = DT_CNTRL;
    while (ni < n_time_cntrl)
    {
        TIME_CNTRL(ni) = xi;
        ni = ni + 1;
        xi = xi + DT_CNTRL;
    }
    if (BE_VERBOSE) std::cout << "\nn_time_cntrl:\n" << n_time_cntrl;

    int n_cntrl = n_vcntrl_lin*n_vcntrl_ang;
    int n_kini = lite::volume(KiniState.size()); // 5: x,y,thet,vlin,vang
    int n_obs = ObstaclesXY_n;
    if (BE_VERBOSE) std::cout << "\nn_cntrl:\n" << n_cntrl;
    if (BE_VERBOSE) std::cout << "\nn_kini:\n" << n_kini;
    if (BE_VERBOSE) std::cout << "\nn_obs:\n" << n_obs;

    // KiniStateTrajCntrl = zeros(n_cntrl, n_kini, n_time_cntrl); // timeseries trajectory of each control option over DT_EVAL
    lite::array<float[1][1][1]> KiniStateTrajCntrl(n_cntrl, n_kini, n_time_cntrl);

    // ObjFuncInputsCntrl = zeros(n_cntrl, 5); // stores control choices &: CntlrLin, CntrlAng, head, dist, vel
    lite::array<float[1][1]> ObjFuncInputsCntrl(n_cntrl, 5);

    // check basic setup of window, return 0,0 if no valid window:
    if (n_cntrl == 0)
    {
        typeLiteArrayVw nullVw = typeLiteArrayVw(0.0f,0.0f);
        return nullVw;
    }

    float v_cntrl_lin;
    float v_cntrl_ang;
    float x_cntrlFinal;
    float y_cntrlFinal;
    float thet_cur_deg;
    float thet_goal_deg;
    float thet_targ_deg;
    float heading;
    float heading_sum;
    float dist;
    float dist_sum = 0.0f;
    float vel;
    float vel_sum;
    float disttmp;
    float dist_min_cntrl;
    float headNorm;
    float distNorm;
    float velNorm;
    float objFuncEval_max;
    int objFuncEval_maxi;
    int iCntrlVec = 0;
    typeLiteArrayKini KiniStateCntrl;
    typeLiteArrayKini KiniCntrlFinal;
    typeLiteArrayXy vw;
    for (int iLin = 0; iLin < n_vcntrl_lin; iLin++)
    {
        for (int iAng = 0; iAng < n_vcntrl_ang; iAng++)
        {
            // Loop for DT_EVAL and compute final pos / trajectory
            v_cntrl_lin = V_CNTRL_LIN(iLin);
            v_cntrl_ang = V_CNTRL_ANG(iAng);
            KiniStateCntrl = KiniState; // initialize state to rover state, update in loop
            vw(0) = v_cntrl_lin;
            vw(1) = v_cntrl_ang; 
            for (int iEval = 0; iEval < n_time_cntrl; iEval++)
            {
                KiniStateCntrl = f_movexy(KiniStateCntrl, vw);
                KiniStateTrajCntrl(iCntrlVec,0,iEval) = KiniStateCntrl(0); // save
                KiniStateTrajCntrl(iCntrlVec,1,iEval) = KiniStateCntrl(1); // save
                KiniStateTrajCntrl(iCntrlVec,2,iEval) = KiniStateCntrl(2); // save
                KiniStateTrajCntrl(iCntrlVec,3,iEval) = KiniStateCntrl(3); // save
                KiniStateTrajCntrl(iCntrlVec,4,iEval) = KiniStateCntrl(4); // save
            }
            KiniCntrlFinal = KiniStateCntrl; // final kini state for cur cntrl choice
            x_cntrlFinal = KiniCntrlFinal(0);
            y_cntrlFinal = KiniCntrlFinal(1);
           
            // Evaluate objective func inputs head, dist, vel and store:
            //
            // - Heading: after cntrl, how aligned with goal?
            //
            thet_cur_deg = toDegree(KiniStateCntrl(2));
            thet_goal_deg = toDegree(
                atan2(  GOAL_Y - y_cntrlFinal,
                        GOAL_X - x_cntrlFinal));
            thet_targ_deg = 0.0f;
            if (thet_goal_deg > thet_cur_deg)
                thet_targ_deg = thet_goal_deg - thet_cur_deg;
            else
                thet_targ_deg = thet_cur_deg - thet_goal_deg;
            heading = 180.0f - thet_targ_deg;
            heading_sum = heading_sum + heading;
            //
            // - Distance: after cntrl, how near nearest obstacle?
            dist_min_cntrl =2.0f;
            float temp = 0.0f;
            float temp2 = 0.0f;
            for (int iObs = 0; iObs<n_obs; iObs++)
            {
                
                disttmp = std::pow(ObstaclesXY(iObs,0) - x_cntrlFinal, 2.0f) +
                          std::pow(ObstaclesXY(iObs,1) - y_cntrlFinal, 2.0f);
                disttmp = std::sqrt(disttmp);
                disttmp = disttmp - RADIUS_OBS;
                //disttmp = std::max(disttmp, 0.1f); // note: limit min dist to 0.1f in case nearer than radius obs
 
                if (dist_min_cntrl>disttmp)
                    dist_min_cntrl=disttmp; // find distance to nearest obstacle
                    temp = x_cntrlFinal;
                    temp2 = y_cntrlFinal;
            }
            dist = dist_min_cntrl;
            dist_sum = dist_sum + dist;
            //
            // - Velocity: simply absolute value of control velocity
            vel = std::abs(v_cntrl_lin);
            vel_sum = vel_sum + vel;
            //
            // Store objective function inputs for later analysis:
            ObjFuncInputsCntrl(iCntrlVec,0) = v_cntrl_lin;            
            ObjFuncInputsCntrl(iCntrlVec,1) = v_cntrl_ang;            
            ObjFuncInputsCntrl(iCntrlVec,2) = heading;            
            ObjFuncInputsCntrl(iCntrlVec,3) = dist;            
            ObjFuncInputsCntrl(iCntrlVec,4) = vel;

            iCntrlVec = iCntrlVec + 1; // increment control pair index
        }
    }
   
    //--------------------------------------------------------------------    
    // Normalize the objective function inputs:
    //  objFunc_inputs_cntrl = [...
    //      x_cntrlFinal, ...
    //      y_cntrlFinal, ...
    //      heading, ...
    //      dist, ...
    //      vel ];    
    for (int i = 0; i < n_cntrl; i++)
    {
            headNorm = ObjFuncInputsCntrl(i,2)/heading_sum;
            distNorm = ObjFuncInputsCntrl(i,3)/dist_sum;
            velNorm = ObjFuncInputsCntrl(i,4)/vel_sum;
            ObjFuncInputsCntrl(i,2) = headNorm;
            ObjFuncInputsCntrl(i,3) = distNorm;
            ObjFuncInputsCntrl(i,4) = velNorm;
    }
    if (BE_VERBOSE) std::cout << "\nObjFuncInputsCntrl: \n" << ObjFuncInputsCntrl;

    //--------------------------------------------------------------------    
    // Evaluation of the objective function:
    lite::array<float[1]> ObjFuncEval(n_cntrl);
    objFuncEval_max = 0.0f;
    objFuncEval_maxi = 0;
    for (int i=0; i < n_cntrl; i++)
    {
        heading = ObjFuncInputsCntrl(i,2);
        dist = ObjFuncInputsCntrl(i,3);
        vel = ObjFuncInputsCntrl(i,4);
        float objFuncEval = 
            WEIGHT_HEAD*heading +
            WEIGHT_DIST*dist +
            WEIGHT_VELO*vel;
        ObjFuncEval(i) = objFuncEval;
        if (objFuncEval > objFuncEval_max)
        {
            objFuncEval_max = objFuncEval;
            objFuncEval_maxi = i;
        }
    }
    // Note: At this point, C & Matlab diverge from rounding
    if (BE_VERBOSE) std::cout << "\nObjFuncEval: \n" << ObjFuncEval;
    if (BE_VERBOSE) std::cout << "\nobjFuncEval_max: \n" << objFuncEval_max;
    if (BE_VERBOSE) std::cout << "\nobjFuncEval_i: \n" << objFuncEval_maxi;
   
    //--------------------------------------------------------------------    
    // Selection of control variables, v_cntrl_lin & v_cntrl_ang:
    v_select_lin = ObjFuncInputsCntrl(objFuncEval_maxi,0);
    v_select_ang = ObjFuncInputsCntrl(objFuncEval_maxi,1);
    if (BE_VERBOSE) std::cout << "\n v_select_lin: \n" << v_select_lin;
    if (BE_VERBOSE) std::cout << "\n v_select_ang: \n" << v_select_ang;
   
    //--------------------------------------------------------------------
    // Output Final Struct containing v_lin, v_ang, and index i

    typeLiteArrayVw OutputVW;
    OutputVW = typeLiteArrayVw(v_select_lin, v_select_ang);
    return OutputVW;
}


void poset265Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // do nothing
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_rad_arb");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Read in parameters to global var's

    
    // Subscribe to input topics
    ros::Subscriber sub_pose = n.subscribe("t265_pose", 1000, poset265Callback);

    // Announce node is running
    ROS_INFO("turn_rad_arb node - RUNNING");

    // TEMP - code to learn lite library for math
    simSetup();
    typeLiteArrayKini KiniStateIni = typeLiteArrayKini(
                                        0.0f,
                                        0.0f,
                                        1.57080f,
                                        0.0f,
                                        0.0f);
    typeLiteArrayVw vw = typeLiteArrayVw(
                                        0.0f,
                                        -0.087266);
    typeLiteArrayVw vwCntrl;

    vwCntrl = f_doDwa(KiniStateIni, ObstaclesXY);
    typeLiteArrayKini KiniStateFinal;
    KiniStateFinal = (typeLiteArrayKini)f_movexy(KiniStateIni, vw);

    while (ros::ok())
    {
        // simSetup();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}