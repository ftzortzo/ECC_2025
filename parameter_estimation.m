function m = parameter_estimation(A, use_regularization)
    % ------------ 1) Preprocessing ----------------
    A = double(A(:));
    M = reshape(A, 6, 10000);

    k_leader   = max(1, min(10000, round(M(5,1) - 1)));
    k_follower = max(1, min(10000, round(M(6,1) - 1)));
    k          = min(k_leader, k_follower);

    if k < 20
        output_vector = [1.6, 4.0, 1.5];  % fallback
        return
    end

    % ---------- 2) Extract raw signals ----------
    leader_position   = M(1,1:k);
    leader_velocity   = M(2,1:k);
    follower_position = M(3,1:k);
    follower_velocity = M(4,1:k);
    dt = 0.02;

    % ---------- 3) Filter out implausible accel ----------
    a_est = diff(follower_velocity) / dt;
    a_min = -6;
    a_max =  5;
    valid_idx = find(a_est > a_min & a_est < a_max) + 1;

    if numel(valid_idx) < 20
        output_vector = [1.6, 4.0, 1.5];
        return
    end

    leader_position   = leader_position(valid_idx);
    leader_velocity   = leader_velocity(valid_idx);
    follower_position = follower_position(valid_idx);
    follower_velocity = follower_velocity(valid_idx);


    % inputs stay unscaled (they're positions/vels as the model expects)
    u = [leader_position(:), leader_velocity(:)];

    %% ---------- 4) NORMALIZE OUTPUT | THIS IS THE PLACE WHERE I SCALE THE OUTPUT ----------
    % scale to bring position from O(10^3) -> O(1..10)
    scale_y = max(1, max(abs(follower_position)));   % prevent 0
    y = follower_position(:) / scale_y;              % <-- scaled output
    %% ---------- 4) END NORMALIZE OUTPUT | THIS IS THE PLACE WHERE I SCALE THE OUTPUT ----------


    if any(~isfinite(u(:))) || any(~isfinite(y(:)))
        output_vector = [1.6, 4.0, 1.5];
        return
    end

    z = iddata(y, u, dt);

    % ---------- 5) Model specification ----------
    order = [1 2 2];  % ny=1, nx=2, nu=2
    x0 = [follower_position(1); follower_velocity(1)];
% params (normalized): v0, T_norm, s0, a_norm, b, delta_norm, Ts, scale_y

    %% ---------- 4) HERE IS THE PLACE I DEFINE WHICH VARIABLES ARE FIXED ----------


P(1) = struct('Name','v0',       'Unit','m/s',  'Value',15.0, ...
              'Minimum',1.0,    'Maximum',60.0, 'Fixed',true);

P(2) = struct('Name','T_norm',   'Unit','-',    'Value',1.3,  ... 
              'Minimum',0.3,    'Maximum',4,  'Fixed',false);

P(3) = struct('Name','s0',       'Unit','m',    'Value',2.0,  ...
              'Minimum',0.1,    'Maximum',10.0, 'Fixed',true);

P(4) = struct('Name','a_norm',   'Unit','-',    'Value',2.0,  ... 
              'Minimum',0.2,    'Maximum',3,  'Fixed',false);

P(5) = struct('Name','b',        'Unit','m/s2', 'Value',3.0,  ...
              'Minimum',0.2,    'Maximum',7.0,  'Fixed',true);

P(6) = struct('Name','delta_norm','Unit','-',   'Value',4.0,  ... 
              'Minimum',0.2,    'Maximum',7.0,  'Fixed',false);

P(7) = struct('Name','Ts',       'Unit','s',    'Value',dt,   ...
              'Minimum',dt,     'Maximum',dt,   'Fixed',true);

P(8) = struct('Name','scale_y',  'Unit','-',    'Value',scale_y, ...
              'Minimum',scale_y,'Maximum',scale_y,'Fixed',true);

    %% ---------- 4) END HERE IS THE PLACE I DEFINE WHICH VARIABLES ARE FIXED ----------


% 2) create model
m0 = idnlgrey('idm_dt_nl', order, P, x0, dt);


    % ---------- 6) Options ----------
    opt = nlgreyestOptions('Display','on');
    opt.SearchMethod = 'lm';  % <- more robust than 'gn'
    opt.SearchOptions.MaxIterations = 20;
   
 %% I USE THE FOLLOWING BLOCK WHEN THE SECOND INPUT IS 1. THIS BLOCK DOES REGULARIZATION TO AVOID OVERFFITING %%   

    if use_regularization
        opt.Regularization.Nominal = 'model';
        % now we are mainly estimating: T, a, delta  -> 3 params
        opt.Regularization.R = diag([0.5, 0.5, 0.5]);  % start simple
        opt.Regularization.Lambda = 0.3;         % a bit softer than 1
    end
 %% END I USE THE FOLLOWING BLOCK WHEN THE SECOND INPUT IS 1. THIS BLOCK DOES REGULARIZATION TO AVOID OVERFFITING %%   

    % ---------- 7) Estimate ----------
    m = nlgreyest(z, m0, opt);

    vals = [m.Parameters.Value];
    % vals = [v0, T, s0, a, b, delta, Ts, scale_y]
    % return [T, delta, a] in that order:
    vals = [m.Parameters.Value];
% vals = [v0, T_norm, s0, a_norm, b, delta_norm, Ts, scale_y]


end
