clc;
clear all;
close all;

%%
% function [...
%     x_cur, ...
%     y_cur, ...
%     thet_cur, ...
%     vlin_cur, ...
%     vang_cur, ...
function [...
    u, ...
    trajDB ...
    ]=DynamicWindowApproach(x,GoalXY,ObjFuncWeights,ob)
%DWA - Function to calculate the lin & ang vel value by
    %Dynamic Window Creation [vmin,vmax,ωmin,ωmax]
   
    global x_cur;
    global y_cur;
    global thet_cur;
    global vlin_cur;
    global vang_cur;    
   
    Vr=CalcDynamicWindow(x);
    % Evaluation function calculation
    [evalDB,trajDB]=Evaluation(x,Vr,GoalXY,ob);

    if isempty(evalDB)
        disp('no path to GoalXY!!');
        u=[0;0];return;
    end

    %各評価関数の正規化
    evalDB=NormalizeEval(evalDB);

    %Calculation of final evaluation value
    feval=[];
    for id=1:length(evalDB(:,1))
        feval=[feval;ObjFuncWeights(1:3)*evalDB(id,3:5)'];
    end
    evalDB=[evalDB feval];

    [maxv,ind]=max(feval);%Calculate the index of the lin & ang vel value with the highest evaluation value
    u=evalDB(ind,1:2)';%Returns lin & ang vel with a high evaluation value
endfunction

function [evalDB,trajDB]=Evaluation(x,Vr,GoalXY,ob)
% A function that calculates the evaluation value for each path
    global RES_V_LIN;
    global RES_V_ANG;
   
    evalDB=[];
    trajDB=[];

    for vt=Vr(1):RES_V_LIN:Vr(2)
        for ot=Vr(3):RES_V_ANG:Vr(4)
            % step through resolution of linear velocity [KinematicLims 5] and angular
            % resolution [KinematicLims 6] from initial vel to final vel lin and ang.
            % For each control option, estimate trajectory and save to trajDB
            % for later use. traj
            %
            % Trajectory estimation
            [xt,traj]=GenerateTrajectory(x,vt,ot);
            %Calculation of each evaluation function
            heading=CalcHeadingEval(xt,GoalXY);
            dist=CalcDistEval(xt,ob);
            vel=abs(vt);

            evalDB=[evalDB;[vt ot heading dist vel]];
            trajDB=[trajDB;traj];    
        end
    end
endfunction

function EvalDB=NormalizeEval(EvalDB)
%Function that normalizes the evaluation value
    if sum(EvalDB(:,3))~=0
        EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
    end
    if sum(EvalDB(:,4))~=0
        EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
    end
    if sum(EvalDB(:,5))~=0
        EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
    end
endfunction

function [x,traj]=GenerateTrajectory(x,vt,ot)
% Function to create trajectory data
    global dt;
    global DT_EVAL;
    time=0;
    u=[vt;ot];%Input Value
    traj=x;%Trajectory data
    while time<=DT_EVAL
        time=time+dt;% Simulation time update
        x=f(x,u);% Translation estimate from control option over DT_EVAL
        traj=[traj x];
    end
endfunction

function stopDist = CalcBreakingDist(vel)
%A function that calculates the braking distance from the current speed according to the dynamic KinematicLims
    global dt;
    global A_MAX_LIN;
    stopDist=0;
    while vel>0
        stopDist=stopDist+vel*dt;%Braking distance calculation
        vel=vel-A_MAX_LIN*dt;% Highest principle
    end
endfunction

function dist=CalcDistEval(x,ob)
% A function that calculates the distance evaluation value with an ObstaclesXY
    global RADIUS_OBS;
   
    dist=2;
    for io=1:length(ob(:,1))
        disttmp=norm(ob(io,:)-x(1:2)')-RADIUS_OBS;%Calculate the norm error between the path position and the ObstaclesXY
        if dist>disttmp % Find the minimum
            dist=disttmp;
        end
    end
endfunction

function heading=CalcHeadingEval(x,GoalXY)
%heading - Function to calculate the evaluation function of

    theta=toDegree(x(3));%Robot orientation
    GoalXYTheta=toDegree(atan2(GoalXY(2)-x(2),GoalXY(1)-x(1)));% GoalXY direction

    if GoalXYTheta>theta
        targetTheta=GoalXYTheta-theta;%Direction difference up to[deg]
    else
        targetTheta=theta-GoalXYTheta;%Direction difference up to[deg]
    end

    heading=180-targetTheta;
endfunction

function Vr=CalcDynamicWindow(x)
% From the KinematicLims and current state calculate DyamicWindow
    global dt;
    global V_MAX_LIN;
    global V_MAX_ANG;
    global A_MAX_LIN;
    global A_MAX_ANG;
   
    %Window Depends on vehicle KinematicLims
    Vs=[0, ...
        V_MAX_LIN, ...
        -V_MAX_ANG, ...
        V_MAX_ANG];

    %Window Depends on the dynamic KinematicLims
    Vd=[x(4)-A_MAX_LIN*dt, ...
        x(4)+A_MAX_LIN*dt, ...
        x(5)-A_MAX_ANG*dt, ...
        x(5)+A_MAX_ANG*dt];

    %Ultimate Dynamic Window Calculation
    Vtmp=[Vs;Vd];
    Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
    %[vmin,vmax,ωmin,ωmax]
endfunction

function KineState = f(x, traj)
% Motion Model
% x: [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] SB: initial state of robot
% traj: trajectory of the control option [x,y]
    global dt;    
    % F*x:
    x_ini = x(1);
    y_ini = x(2);
    thet_ini = x(3);
    v_ini_lin = x(4);
    v_ini_ang = x(5);
   
    % F*x:
    % x_ini
    % y_ini
    % yaw_ini
    % 0_??note: is 0 here, because taken directly from traj(1) aka vlin_cur ???
    % 0 = a_ini
   

    %     traj:
    %     vlin_cur              
    %     vang_cur
    %    
    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];    
    % B:
    % dt*cos(thet_cur)    ?
    % dt*sin(thet_cur)    ?
    % ?                   dt
    % 1                   ?
    % ?                   1
    %
    %
    % B*traj:
    % vlin_cur*cos(thet_cur)*dt = x_delta = "x comp of vlin"
    % vlin_cur*sin(thet_cur)*dt = y_delta = "y comp of vlin"
    % vang_cur*dt = theta_delta = "delta theta by constant ang vel * dt"
    % vlin_cur = vlin_delta
    % vang_cur = vang_delta

    % KineState:
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
   
    x_delta = v_cur_lin*cos(thet_ini)*dt;
    y_delta = v_cur_lin*sin(thet_ini)*dt;
    thet_delta = v_cur_ang*dt;
   
    x_final = x_ini + x_delta;
    y_final = y_ini + y_delta;
    thet_final = thet_ini + thet_delta;
   
    v_final_lin = v_cur_lin; % assume car reaches command at end
    v_final_ang = v_cur_ang; % assume car reaches command at end
   
    KineState = [...
        x_final, ...
        y_final, ...
        thet_final, ...
        v_final_lin, ...
        v_final_ang ]';
       
endfunction

function radian = toRadian(degree)
    % degree to radian
    radian = degree/180*pi;
endfunction

function degree = toDegree(radian)
    % radian to degree
    degree = radian/pi*180;
endfunction



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
% Given X,Y global coordinate frame; set of ObstaclesXYs OBS, and objective
% position TARG; determine pair of target linear & angular velocities v & w
% for the next time step that maximize an objective function capturing:
% max/min velocity, max/min acceleration, distance to ObstaclesXYs, heading
% relative to target.
%
%
% References:
% https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
% https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
% http://adrianboeing.blogspot.com/2012/05/dynamic-window-algorithm-motion.html
% https://reader.elsevier.com/reader/sd/pii/S1474667015343603?token=9CC9D5C8E51883E1910E3E5D6E4CCECAA729DC3D97AB87FD5D8AAC93C6A8E2D5C22E1D88FBFE3AC099B0CA976BC3C0F1&originRegion=us-east-1&originCreation=20211005210438

% global state var's:
global x_cur;
global y_cur;
global thet_cur;
global vlin_cur;
global vang_cur;

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
global WEIGHT_DIST;
global WEIGHT_HEAD;
global WEIGHT_VELO;
global AREA_SIM;
%%
A_MAX_ANG = toRadian(50);   % rad/s/s
A_MIN_ANG = toRadian(-50);  % rad/s/s
A_MAX_LIN = 9.81*0.3;       % m/s/s
A_MIN_LIN = -9.81*0.3;      % m/s/s
V_MAX_ANG = toRadian(20);   % rad/s
V_MIN_ANG = toRadian(20);   % rad/s
V_MAX_LIN = 30*1000/3600;   % m/s
V_MIN_LIN = -5*1000/3600;   % m/s
RES_V_LIN = 0.01;           % m/s, control resolution, search space size
RES_V_ANG = toRadian(1);    % rad, control resolution, search space size
RADIUS_OBS = 0.25;          % m, assume ObstaclesXY extends this radially
DT_CNTRL = 0.4;             % s, step size for control ie 1/hz loop rate
DT_EVAL = 3.0;              % s, model traj for cntrl option v,w this long
WEIGHT_DIST = 0.1;          % objective func weight, dist to ObstaclesXY
WEIGHT_HEAD = 0.4;          % objective func weight, heading wrt GoalXY
WEIGHT_VELO = 0.2;          % objective func weight, linear velocity
AREA_SIM = [-1 30 -1 30]    % m, simulation area, xmin xmax ymin ymax
%%
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
WEIGHT_DIST = 0.1;          % objective func weight, dist to ObstaclesXY
WEIGHT_HEAD = 0.2;          % objective func weight, heading wrt GoalXY
WEIGHT_VELO = 0.2;          % objective func weight, linear velocity
AREA_SIM = [-1 11 -1 11]    % m, simulation area, xmin xmax ymin ymax
%%
% Initial conditions for simulation
WZERO = 1; % w deg/s less than this treated as zero
THETZERO = 1; % theta deg less than this treated as zero

V_INI_LIN = 1.0; % m/s, T_INIial velocity at start of path planning
V_INI_ANG = toRadian(0.0); % rad/s/s, T_INIial rotaional velocity, CCW pos
T_INI = toRadian(90.0); % rad, T_INIial heading, CCW pos
X_INI = 0.0; % T_INIial x coord
Y_INI = 0.0; % T_INIial y coord
X_GoalXY = 14;
Y_GoalXY = 9;
%%
% Initial conditions for simulation
WZERO = 1; % w deg/s less than this treated as zero
THETZERO = 1; % theta deg less than this treated as zero

V_INI_LIN = 0.0; % m/s, T_INIial velocity at start of path planning
V_INI_ANG = toRadian(0.0); % rad/s/s, T_INIial rotaional velocity, CCW pos
T_INI = toRadian(90.0); % rad, T_INIial heading, CCW pos
X_INI = 0.0; % T_INIial x coord
Y_INI = 0.0; % T_INIial y coord

X_GoalXY = 10;
Y_GoalXY = 10;

%%
 
disp('Dynamic Window Approach sample program start!!')

KineState =[X_INI ...
            Y_INI ...
            T_INI ...
            V_INI_LIN ...
            V_INI_ANG ...
            ]';%ロボットの初期状態[x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)] SB: initial state of robot
       

GoalXY=[X_GoalXY,Y_GoalXY]; % [x(m),y(m)] SB: GoalXY position

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
     

global dt;
dt=DT_CNTRL;% [s] SB: Step time

% SB:
% Robot dynamic KinematicLims:
%Maximum speed[m/s],Maximum turning back speed[rad/s],Maximum acceleration and deceleration[m/ss],Maximum plus and minus turning back speed[rad/ss],
% Speed resolution [m/s],Turn back speed resolution[rad/s]]
KinematicLims = [   V_MAX_LIN, ...
                    V_MAX_ANG, ...
                    A_MAX_LIN, ...
                    A_MAX_ANG, ...
                    RES_V_LIN, ...
                    RES_V_ANG ];

%Evaluation function parameters [heading,dist,velocity,predictDT]
ObjFuncWeights=[WEIGHT_HEAD, WEIGHT_DIST, WEIGHT_VELO];

% simulation result
result.KineState = [];

tic;

% Main loop
for i=1:5000
    %Calculation of input value by DWA
    [...
%     x_cur, ...
%     y_cur, ...
%     thet_cur, ...
%     vlin_cur, ...
%     vang_cur, ...
    u, ...
    traj ...
    ] = DynamicWindowApproach(KineState, GoalXY, ObjFuncWeights, ObstaclesXY);
    %[u,traj]=DynamicWindowApproach(KineState, GoalXY, ObjFuncWeights, ObstaclesXY);
%         u = [...
%         vlin_cur, ...
%         vang_cur ...
%         ]
    KineState = f(KineState,u);%Movement by motion model
   
    %Saving simulation results
    result.KineState=[result.KineState; KineState'];
   
    %GoalXY judgment
    if norm(KineState(1:2)-GoalXY')<0.5
        disp('Arrive GoalXY!!');break;
    end
   
    %====Animation====
    hold off;
    ArrowLength=0.5;%矢印の長さ
    %ロボット
    quiver(KineState(1),KineState(2),ArrowLength*cos(KineState(3)),ArrowLength*sin(KineState(3)),'ok');hold on;
    plot(result.KineState(:,1),result.KineState(:,2),'-b');hold on;
    plot(GoalXY(1),GoalXY(2),'*r');hold on;
    plot(ObstaclesXY(:,1),ObstaclesXY(:,2),'*k');hold on;
    % Exploration trajectory representation
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(AREA_SIM);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% アニメーションのフレームをゲットする
end
figure(2)
plot(result.KineState(:,4));
toc
%movie2avi(mov,'movie.avi');
 

