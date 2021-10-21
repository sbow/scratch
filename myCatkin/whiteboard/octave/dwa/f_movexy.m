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