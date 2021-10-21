% sim.m
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

