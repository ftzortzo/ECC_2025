function IDM_params = enhanced_parameter_estimation(leader_position,leader_velocity,follower_position,follower_velocity,k_leader,k_follower,llead,dt)
    %% Filippos Notes
    %K is the index untill where we have data in the vectors Xbuf and Ybuf.
    %That is Xbuf can be [13,14,15,16,0,0,0,0]. Then K=4.
    % position data include position data where position is measured with
    % respect to the entry of the buffer zone.
    % position_data_of_preceding_vehicle correspond to the position of the
    % preceding vehicle at the same time steps
    %Here we try to lean parameters (we will decide which ones) from the idm
    %model.
    %parameters will be the vector parameters
    
    %% Parameter estimation function for Simulink
    % Inputs:
    %   leader_position: array of leader pos values
    %   leader_velocity: array of leader vel values
    %   follower_position: array of follower pos values
    %   follower_velocity: array of follower vel values
    %   k_leader: index indicating valid data length of leader (data is valid from 1 to k)
    %   k_follower: index indicating valid data length of follower (data is valid from 1 to k)
    %   llead: length of the leader vehicle
    %   dt: simulation timestep
    % Outputs:
    %   IDM_a: estimated acceleration parameter
    %   IDM_b: estimated deceleration parameter
    
    % setting some default value for IDM parameters incase estimate of IDM fails
    IDM_a = 2.00;
    IDM_b = 2.00;
    T_head = 2.00;
    delta = 2.00;
    
    % compare matching indecies in the buffer zone
    k = min(k_leader,k_follower);
    if k < 10
        warning('10 or fewer samples for prediction')
    end
    
    %% IDM Estimation
    try
        % Format the data for the IDM
        leader_accel = [0; diff(leader_velocity(1:k))/dt];
        u = [leader_position(1:k),leader_velocity(1:k),leader_accel];
        y = follower_velocity(1:k);
        ego_pos = follower_position(1:k);
    
        % creating the iddata object that grey box requires
        z = iddata(y, u, dt);
    
        % model order
        order = [1 3 2];
    
        % initalize states and params
        headway = 1.6; %seconds
        standstill = 2.0; %meters
        vEGO0 = y(1);
        xEGO0 = ego_pos(1);
        x0 = [xEGO0;vEGO0];
    
        % Model parameters
        P(1) = struct('Name','v0','Unit','m/s','Value',25,'Minimum',0.1,'Maximum',60,'Fixed',true);
        P(2) = struct('Name','T_h','Unit','s','Value',1.6,'Minimum',0.5,'Maximum',4,'Fixed',false);
        P(3) = struct('Name','s0','Unit','m','Value',2.0,'Minimum',0.1,'Maximum',8,'Fixed',true);
        P(4) = struct('Name','a','Unit','m/s^2','Value',1.0,'Minimum',1.0,'Maximum',5,'Fixed',false);
        P(5) = struct('Name','b','Unit','m/s^2','Value',3.0,'Minimum',1.0,'Maximum',5,'Fixed',false);
        P(6) = struct('Name','delta','Unit','','Value',4.0,'Minimum',2,'Maximum',6,'Fixed',false);
        P(7) = struct('Name','llead','Unit','m','Value',llead,'Minimum',3,'Maximum',8,'Fixed',true);
        P(8) = struct('Name','Ts','Unit','s','Value',dt,'Minimum',dt,'Maximum',dt,'Fixed',true);
        P(9) = struct('Name','c','Unit','','Value',0.99,'Minimum',0.95,'Maximum',1.0,'Fixed',true);
    
        % build greybox idnlgrey
        m0 = idnlgrey(@idm_dt_nl, order, P, x0, dt, 'Name','IDM_dt'); % I was trying to use @idm_dt_nl_local but seemed to not work...? idm_dt_nl
        m0 = setinit(m0,'Fixed',[false; false]);
        opt = nlgreyestOptions('Display','off');
        opt.SearchMethod = 'gn'; %gaussian search method
        opt.SearchOption.MaxIterations = 15;
        opt.SearchOption.Tolerance = 1e-4; %stopping loss function thd for gaussian to try and tune perf vs acc%
        opt.OutputWeight = [];
       
    
        % estimate
        m = nlgreyest(z, m0, opt);
    
        % extract values
        vals = [m.Parameters.Value];
        IDM_a = vals(4);
        IDM_b = vals(5);
        T_head = vals(2);
        delta = vals(6);
        IDM_params = [IDM_a,IDM_b,T_head,delta];
    catch ME
        warning('Parameter estimation failed: %s', ME.message);
    
    
    end
end

%% helper functions used in above
% this is the discrete timestep implementation of IDM
function [xnext, y] = idm_dt_nl(~, x, u, v0, T_h, s0, a, b, c, delta, llead, Ts, varargin)
    % States: x=[xE; vE]; Inputs: u=[xL; vL]; Output: y=vE
    xE = x(1);
    vE = x(2);
    xL = u(1); 
    vL = u(2);
    aL = u(3);
    
    % gap and closing rate
    s  = xL - xE - llead; %gap
    dv = vE - vL;
    
    % IDM using continuius idm to get discrete time state update for sys id toolbox
    p = struct('v0',v0,'T',T_h,'s0',s0,'a',a,'b',b,'delta',delta);
    a_IDM = idm_dvdt(vE, max(s,1e-3), dv, p);

    % Calculate the CAH enhanced IDM acceleration
    a_CAH = cah_accel(s, vE, vL, aL, a);
    
    % Calculate the ACC enhanced IDM acceleration
    acc = acc_accel(a_IDM, a_CAH, b, c);
    
    % Euler step
    vE2 = max(0, vE + Ts*acc);
    xE2 = xE + Ts*vE;
    
    xnext = [xE2; vE2];
    y = vE2;
end

% this is the continious implementation of IDM that the discrete function above uses to calculate
function acc = idm_dvdt(v_a, s_a, dv_a, p)
% IDM acceleration dot v_alpha for follower alpha
% Inputs:
% v_a   : v_alpha  (speed, m/s)
% s_a   : s_alpha  (bumper-to-bumper gap to leader, m)
% dv_a  : Delta v_alpha = v_alpha - v_{lead}  (approaching rate, m/s)
% p     : struct with fields v0, T, s0, a, b, delta
% Output:
% acc    : dot v_alpha  (acceleration, m/s^2)

    % ---- pull out parameters from array ----
    v0 = p.v0; % v_0: desired speed (free-flow speed), m/s
    T  = p.T; % T: desired time headway, s
    s0 = p.s0; % s_0: minimum standstill gap, m
    a = p.a; % a: maximum acceleration parameter, m/s^2
    b = p.b; % b: comfortable deceleration parameter, m/s^2
    if isfield(p,'delta')
        delta = p.delta;
    else
        delta = 4;
    end


    % ---- free-road term (v_alpha / v_0)^delta ----
    phi_free = (v_a / max(v0, 1e-6))^delta; % protect against v0=0

    % ----- desired dynamic gap s*(v_alpha,Delta v_alpha) for use in interaction term
    s_star = s0 + v_a*T + (v_a*dv_a)/(2*sqrt(a*b));

    % ---- interaction term (s* / s_alpha)^2 ----
    s_eff = max(s_a, 1e-3); % avoid divide-by-zero for tiny gaps - where cars would be touching?
    phi_int = (max(s_star,0) / s_eff)^2; % s* must be greater than zero

    % ---- IDM acceleration: dot v_alpha = a [ 1 - (v/v0)^delta - (s*/s)^2 ] ----
    acc = a * (1 - phi_free - phi_int);
end

function a_CAH = cah_accel(s, vE, vL, aL, a)
    % Inputs
    % s: gap
    % v: follower velocity
    % v_l: leader velocity
    % a_l: leader acceleration
    % a: max accel of follower

    acc_final = min(aL,a);
    dv = vE-vL;

    if vL*dv <= -2*s*acc_final %this is from the enhanced IDM papaer which determines which CAH equation to use and figures if vehicles will stop before reaching gap = 0
        if abs(vL) < 0.000001
            % here we have the condition that both vehicles stop
            a_CAH = 0;
        else
            % here we have the condition that vehilces are still moving at the min gap
            a_CAH = vE^2 * acc_final / (vL^2 - 2*s*acc_final);
        end
    else
        dv_2 = double(dv > 0);
        a_CAH = acc_final - (dv^2 * dv_2)/(2*max(s,0.000001));
    end
end

function acc = acc_accel(a_IDM,a_CAH, b, c)
    % Inputs
    % a_IDM: IDM Accel
    % a_CAH: CAH Accel
    % b: comfy deceleration
    % c: coolness factor

    if a_IDM >= a_CAH
        acc = a_IDM;
    else
        acc = (1-c) * a_IDM + c * (a_CAH + b * tanh((a_IDM - a_CAH)/b));
    end
end

