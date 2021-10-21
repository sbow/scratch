% sim_doDwa.m
% v0.5
% 2021/10/12
% By Shaun Bowman
%
%
% Description:
% Matlab implimentation of Dynamic Window Approuch for Path Planning
% With: fixed set of W,V pairs to try; calibration style lookup parameters
% for max accel as a function of host vehicle speed.
%
% Objective:
% Given X,Y global coordinate frame; set of obstacles OBS, and objective
% position TARG; determine pair of target linear & angular velocities v & w
% for the next time step that maximize an objective function capturing:
% max/min velocity, max/min acceleration, distance to obstacles, heading
% relative to target.
%
%
% References:
% https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
% https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
% http://adrianboeing.blogspot.com/2012/05/dynamic-window-algorithm-motion.html
% https://reader.elsevier.com/reader/sd/pii/S1474667015343603?token=9CC9D5C8E51883E1910E3E5D6E4CCECAA729DC3D97AB87FD5D8AAC93C6A8E2D5C22E1D88FBFE3AC099B0CA976BC3C0F1&originRegion=us-east-1&originCreation=20211005210438
clc;
clear all;
close all;

%%
%% sim veh motion / world model
function KiniState = f_movexy(x, traj)
% Motion Model
% x: [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] SB: initial state of robot
% traj: trajectory of the control option [x,y]
    global DT_CNTRL;    
    % F*x:
    % x_ini
    % y_ini
    % yaw_ini
    % 0_??note: is 0 here, because taken directly from traj(1) aka vlin_cur ???
    % 0 = a_ini

    %     traj:
    %     vlin_cur
    %     vang_cur
 
    % B:
    % DT_CNTRL*cos(thet_cur)    ?
    % DT_CNTRL*sin(thet_cur)    ?
    % ?                   DT_CNTRL
    % 1                   ?
    % ?                   1
    %
    % B*traj:
    % vlin_cur*cos(thet_cur)*DT_CNTRL = x_delta = "x comp of vlin"
    % vlin_cur*sin(thet_cur)*DT_CNTRL = y_delta = "y comp of vlin"
    % vang_cur*DT_CNTRL = theta_delta = "delta theta by constant ang vel * DT_CNTRL"
    % vlin_cur = vlin_delta
    % vang_cur = vang_delta

    % KiniState:
    % X_CUR = x_ini + x_delta
    % Y_CUR = y_ini + y_delta
    % T_CUR = t_ini + t_delta
    % V_CUR_LIN = v_ini + v_delta = 0 + v_delta = v_delta = vlin_cur =
    % vlin_control : note, implicitly assumes the vehicle simply achieves
    % the angular acceleration and velocity of the control signal. There is
    % no actual model of vehicle velocity; linear or angular.
    % This is essentially just saying
    % that on the next control loop, the linear vel / accel equals the
    % control system's commands. It does however model positoin and
    % heading.
    % V_CUR_ANG = vang_delta = vang_cur = vang_control

    x_ini = x(1);
    y_ini = x(2);
    thet_ini = x(3);
    v_ini_lin = x(4);
    v_ini_ang = x(5);
   
    v_cur_lin = traj(1);
    v_cur_ang = traj(2);
   
    x_delta = v_cur_lin*cos(thet_ini)*DT_CNTRL;
    y_delta = v_cur_lin*sin(thet_ini)*DT_CNTRL;
    thet_delta = v_cur_ang*DT_CNTRL;
   
    x_final = x_ini + x_delta;
    y_final = y_ini + y_delta;
    thet_final = thet_ini + thet_delta;
   
    v_final_lin = v_cur_lin; % assume car reaches command at end
    v_final_ang = v_cur_ang; % assume car reaches command at end
   
    KiniState = [...
        x_final, ...
        y_final, ...
        thet_final, ...
        v_final_lin, ...
        v_final_ang ]';
       
endfunction

%%
function [  v_select_lin, ...
            v_select_ang, ...
            objFuncEval_maxi, ...
            KiniStateTrajCntrl ...
            ] = f_doDwa(...
            KiniState, ...
            ObstaclesXY ...
            )

    %DWA - Function to calculate the lin & ang vel value by
   
    v_select_lin = 0; % placeholder, control choice of v linear m/s
    v_select_ang = 0; % placeholder, control choice of v angular rad/s
    objFuncEval_maxi = 1; % placeholder, control choice amoung v pairs
   
    %--------------------------------------------------------------------
    %Dynamic Window Creation:
    %
    %[vmin,vmax,ωmin,ωmax]
    % determine max / min v_lin and v_ang based on accel limitations and
    % current velocity
    global V_MAX_LIN;
    global V_MAX_ANG;
    global A_MAX_LIN;
    global A_MAX_ANG;
    global DT_CNTRL;
   
    %Vs : Velocity window - based on robot propulsion limits
    Vs=[0, ...
        V_MAX_LIN, ...
        -V_MAX_ANG, ...
        V_MAX_ANG];
   
    % Vd : Velocity window - ability to stop at max decel in next loop.
    % note: i believe this should be DT_EVAL (ie: ability to stop within
    % next PLANNING phase, not control loop... far too limiting?
    % KiniState = [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] initial state of robot
    v_cur_lin = KiniState(4);
    v_cur_ang = KiniState(5);
    Vd=[v_cur_lin-A_MAX_LIN*DT_CNTRL, ... % TOTRY: DT_EVAL
        v_cur_lin+A_MAX_LIN*DT_CNTRL, ... % TOTRY: DT_EVAL
        v_cur_ang-A_MAX_ANG*DT_CNTRL, ... % TOTRY: DT_EVAL
        v_cur_ang+A_MAX_ANG*DT_CNTRL];     % TOTRY: DT_EVAL
   
    % Arb above to gen control space limits for v & w:
    v_limmin_lin = max(Vs(1), Vd(1));
    v_limmax_lin = min(Vs(2), Vd(2));
    v_limmin_ang = max(Vs(3), Vd(3));
    v_limmax_ang = min(Vs(4), Vd(4));
   
    %--------------------------------------------------------------------    
    % Loop through control options:
    %
    % For each option:
% Loop for DT_EVAL time and compute expected final position / trajectory
% Determine the heading angle between position AFTER move & goal.
% Determine distance between final position and NEAREST object, note: this
%     imposes a constraint, interim points are not checked for distance to
%     obstacle. So if vel is large or DT_CNTRL is large, u might skip past a
%     point where host was very close to obstacle.
% Compute objective function using weighting params.
% Store results of objective function, and trajectories, for later analysis
    global RES_V_LIN;
    global RES_V_ANG;
    global GOAL_X;
    global GOAL_Y;
    global RADIUS_OBS;
    global DT_EVAL;
   
    V_CNTRL_LIN = v_limmin_lin:RES_V_LIN:v_limmax_lin;
    n_vcntrl_lin = length(V_CNTRL_LIN);
    V_CNTRL_ANG = v_limmin_ang:RES_V_ANG:v_limmax_ang;
    n_vcntrl_ang = length(V_CNTRL_ANG);
    TIME_CNTRL = DT_CNTRL:DT_CNTRL:DT_EVAL; % time series for evaluating control options
    n_time_cntrl = length(TIME_CNTRL);
    n_cntrl = n_vcntrl_lin*n_vcntrl_ang;
    n_kini = length(KiniState); % 5: x,y,thet,vlin,vang
    n_obs = length(ObstaclesXY);
    KiniStateTrajCntrl = zeros(n_cntrl, n_kini, n_time_cntrl); % timeseries trajectory of each control option over DT_EVAL
    ObjFuncInputsCntrl = zeros(n_cntrl, 5); % stores control choices &: CntlrLin, CntrlAng, head, dist, vel
    heading_sum = 0;
    dist_sum = 0;
    vel_sum = 0;
   
    % check basic setup of window, return 0,0 if no valid window:
    if n_cntrl == 0
        return;
    end
       
    iCntrlVec = 1;
    for iLin = 1:n_vcntrl_lin
        for iAng = 1:n_vcntrl_ang
           
            % Loop for DT_EVAL and compute final pos / trajectory
            v_cntrl_lin = V_CNTRL_LIN(iLin);
            v_cntrl_ang = V_CNTRL_ANG(iAng);
            KiniStateCntrl = KiniState; % initialize state to rover state, update in loop
            for iEval = 1:n_time_cntrl
                time = TIME_CNTRL(iEval);
                KiniStateCntrl = f_movexy(KiniStateCntrl, [v_cntrl_lin, v_cntrl_ang]);
                KiniStateTrajCntrl(iCntrlVec,:,iEval) = KiniStateCntrl; % save
            end
            KiniCntrlFinal = KiniStateCntrl; % final kini state for cur cntrl choice
            x_cntrlFinal = KiniCntrlFinal(1);
            y_cntrlFinal = KiniCntrlFinal(2);
           
            % Evaluate objective func inputs head, dist, vel and store:
            %
            % - Heading: after cntrl, how aligned with goal?
            %
            thet_cur_deg = toDegree(KiniStateCntrl(3));
            thet_goal_deg = toDegree(...
                atan2(  GOAL_Y - y_cntrlFinal, ...
                        GOAL_X - x_cntrlFinal));
            thet_targ_deg = 0;
            if thet_goal_deg > thet_cur_deg
                thet_targ_deg = thet_goal_deg - thet_cur_deg;
            else
                thet_targ_deg = thet_cur_deg - thet_goal_deg;
            end
            heading = 180 - thet_targ_deg;
            heading_sum = heading_sum + heading;
            %
            % - Distance: after cntrl, how near nearest obstacle?
            dist_min_cntrl =2;
            for iObs = 1:n_obs
                disttmp=norm(ObstaclesXY(iObs,:)-[x_cntrlFinal, y_cntrlFinal]) - RADIUS_OBS; % wtf... norm?
                if dist_min_cntrl>disttmp
                    dist_min_cntrl=disttmp;
                end
            end
            dist = dist_min_cntrl;
            dist_sum = dist_sum + dist;
            %
            % - Velocity: simply absolute value of control velocity
            vel = abs(v_cntrl_lin);
            vel_sum = vel_sum + vel;
            %
            % Store objective function inputs for later analysis:
            objFunc_inputs_cntrl = [...
                v_cntrl_lin ...
                v_cntrl_ang ...
                heading ...
                dist ...
                vel ];
            ObjFuncInputsCntrl(iCntrlVec,:) = objFunc_inputs_cntrl;            

            iCntrlVec = iCntrlVec + 1; % increment control pair index
        end
    end
   
    %--------------------------------------------------------------------    
    % Normalize the objective function inputs:
    %  objFunc_inputs_cntrl = [...
    %      x_cntrlFinal, ...
    %      y_cntrlFinal, ...
    %      heading, ...
    %      dist, ...
    %      vel ];    
    for i = 1:n_cntrl
            headNorm = ObjFuncInputsCntrl(i,3)/heading_sum;
            distNorm = ObjFuncInputsCntrl(i,4)/dist_sum;
            velNorm = ObjFuncInputsCntrl(i,5)/vel_sum;
            ObjFuncInputsCntrl(i,3) = headNorm;
            ObjFuncInputsCntrl(i,4) = distNorm;
            ObjFuncInputsCntrl(i,5) = velNorm;
    end
   
    %--------------------------------------------------------------------    
    % Evaluation of the objective function:
    global WEIGHT_DIST;
    global WEIGHT_HEAD;
    global WEIGHT_VELO;
    ObjFuncEval = zeros(n_cntrl,1);
    objFuncEval_max = 0;
    objFuncEval_maxi = 1;
    for i = 1:n_cntrl
        head = ObjFuncInputsCntrl(i,3);
        dist = ObjFuncInputsCntrl(i,4);
        vel = ObjFuncInputsCntrl(i,5);
        objFuncEval = ...
            WEIGHT_HEAD*head + ...            
            WEIGHT_DIST*dist + ...
            WEIGHT_VELO*vel;
        ObjFuncEval(i) = objFuncEval;
        if objFuncEval > objFuncEval_max
            objFuncEval_max = objFuncEval;
            objFuncEval_maxi = i;
        end
    end
   
    %--------------------------------------------------------------------    
    % Selection of control variables, v_cntrl_lin & v_cntrl_ang:
    v_select_lin = ObjFuncInputsCntrl(objFuncEval_maxi,1);
    v_select_ang = ObjFuncInputsCntrl(objFuncEval_maxi,2);
   
    %--------------------------------------------------------------------
endfunction

function radian = toRadian(degree)
    % degree to radian
    radian = degree/180*pi;
endfunction

function degree = toDegree(radian)
    % radian to degree
    degree = radian/pi*180;
endfunction

%% ----------------------------------------------------------------------
%% ----------------------------------------------------------------------

%%
% calibrations:
global A_MAX_ANG;
global A_MIN_ANG;
global A_MAX_LIN;
global A_MIN_LIN;
global V_MAX_ANG;
global V_MIN_ANG;
global V_MAX_LIN;
global V_MIN_LIN;
global RES_V_LIN;
global RES_V_ANG;
global RADIUS_OBS;
global DT_CNTRL;
global DT_EVAL;
global WEIGHT_HEAD;
global WEIGHT_DIST;
global WEIGHT_VELO;
global AREA_SIM;
global GOAL_X;
global GOAL_Y;
global RAD_GOAL_MET;

A_MAX_ANG = toRadian(50);   % rad/s/s
A_MIN_ANG = toRadian(-50);  % rad/s/s
A_MAX_LIN = 0.2;       % m/s/s
A_MIN_LIN = -0.2;      % m/s/s
V_MAX_ANG = toRadian(20);   % rad/s
V_MIN_ANG = toRadian(20);   % rad/s
V_MAX_LIN = 1;   % m/s
V_MIN_LIN = -1;   % m/s
RES_V_LIN = 0.01;           % m/s, control resolution, search space size
RES_V_ANG = toRadian(1);    % rad, control resolution, search space size
RADIUS_OBS = 1.5;          % m, assume ObstaclesXY extends this radially
DT_CNTRL = 0.1;             % s, step size for control ie 1/hz loop rate
DT_EVAL = 3.0;              % s, model traj for cntrl option v,w this long
WEIGHT_HEAD = 0.2;          % objective func weight, heading wrt GoalXY
WEIGHT_DIST = 0.1;          % objective func weight, dist to ObstaclesXY
WEIGHT_VELO = 0.2;          % objective func weight, linear velocity
AREA_SIM = [-1 15 -1 15];    % m, simulation area, xmin xmax ymin ymax
GOAL_X = 10;    % m, x coordinate of desired final position
GOAL_Y = 10;    % m, y coordinate of desired final position
RAD_GOAL_MET = 0.5; %m, if norm of distance is nearer than this to goal, control sez success

%%
% Initial conditions for simulation
V_INI_LIN = 0.0; % m/s, T_INIial velocity at start of path planning
V_INI_ANG = toRadian(0.0); % rad/s/s, T_INIial rotaional velocity, CCW pos
T_INI = toRadian(90.0); % rad, T_INIial heading, CCW pos
X_INI = 0.0; % T_INIial x coord
Y_INI = 0.0; % T_INIial y coord
N_SIM = 5000; % number of steps in simulation

% [x(m) y(m)] SB: ObstaclesXY list
ObstaclesXY=    [   0 2;
                    4 2;
                    4 4;
                    5 4;
                    5 5;
                    5 6;
                    5 9;
                    8 8;
                    8 9;
                    7 9];

%% Do Sim

% T_INIIALIZE
V_OPTN_LIN =    (V_MIN_LIN:RES_V_LIN:V_MAX_LIN)'; % m/s, broad set of control options, linear velocity
V_OPTN_ANG =    (V_MIN_ANG:RES_V_ANG:V_MAX_ANG)'; % rad/s, broad set of control options, linear velocity
KiniStateIni =  [X_INI; Y_INI; T_INI; V_INI_LIN; V_INI_ANG];
KiniStateCur =  KiniStateIni;
rec.iMaxObjctv = zeros(N_SIM, 1);
rec.x = zeros(N_SIM, 1);
rec.y = zeros(N_SIM, 1);
rec.v_select_lin = zeros(N_SIM, 1);
rec.v_select_ang = zeros(N_SIM, 1);
rec.KiniStateCur = zeros(N_SIM, length(KiniStateCur));

% MAIN LOOP
tic;
for iSim = 1:N_SIM
    % OBSERVE

    % CONTROL
    if norm(KiniStateCur(1:2)-[GOAL_X, GOAL_Y]')<RAD_GOAL_MET %GoalXY judgment
        disp('Arrive GoalXY!!');
        break;
    end
   
    [ v_select_lin, ...
        v_select_ang, ...
        objFuncEval_maxi, ...
        KiniStateTrajCntrl ] = ...
            f_doDwa(...
                KiniStateCur, ...
                ObstaclesXY );
           
    % MOVE
    % get delta x,y for ALL CHOICES & update using selected choice
    KiniStateCur = f_movexy( KiniStateCur, [v_select_lin; v_select_ang]);
    x = KiniStateCur(1);
    y = KiniStateCur(2);

    % RECORD
    rec.iMaxObjctv = objFuncEval_maxi;
    rec.x(1) = x;
    rec.y(1) = y;
    rec.v_select_lin(iSim) = v_select_lin;
    rec.v_select_ang(iSim) = v_select_ang;
rec.KiniStateCur(iSim,:) = KiniStateCur;

    % ANIMATION
    hold off;
    ArrowLength=0.5;
    quiver(KiniStateCur(1),KiniStateCur(2),ArrowLength*cos(KiniStateCur(3)),ArrowLength*sin(KiniStateCur(3)),'ok');hold on;
    plot(KiniStateCur(1),KiniStateCur(2),'-b');hold on;
    plot(GOAL_X,GOAL_Y,'*r');hold on;
    plot(ObstaclesXY(:,1),ObstaclesXY(:,2),'*k');hold on;
    % Exploration trajectory representation
    if ~isempty(KiniStateTrajCntrl)
        % KiniStateTrajCntrl = zeros(n_cntrl, n_kini, n_time_cntrl); %
        % timeseries trajectory of control options in dynamic window
        [n_cntrl, n_kini, n_time_cntrl] = size(KiniStateTrajCntrl);
        for iCntr = 1:n_cntrl
            x_Cntrl = reshape(KiniStateTrajCntrl(iCntr,1,:), numel(KiniStateTrajCntrl(iCntr,1,:)), 1);
            y_Cntrl = reshape(KiniStateTrajCntrl(iCntr,2,:), numel(KiniStateTrajCntrl(iCntr,2,:)), 1);
            plot(x_Cntrl,y_Cntrl,'-g');hold on;
        end
    end
    axis(AREA_SIM);
    grid on;
    drawnow;
end
figure(2)
plot(rec.KiniStateCur(:,4));
toc

%% Display Results

% figure;
% hold on;
% xlim([0, 20]);
% ylim([0, 10]);
% plot(GOAL_X, GOAL_Y, "*r");
% plot(rec.x, rec.y, "*g");

