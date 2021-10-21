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