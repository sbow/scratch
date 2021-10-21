clc;
clear all;
close all;
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
