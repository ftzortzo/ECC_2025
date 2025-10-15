

% In the following code I try to simulate human driven vehicle so that we
% can play with Bayesian Linear regression and see how it works. Because
% here we do not use VR, we need to generate vehicle trajectories (mimicing
% humans) in a different way. To do that, we use a car-following model.
% This model is the IDM model (you can google it). Here We will do a simple
% example considering only a single road. Not a merging. Next we initialize
% some parameters for our simulation. 

Simulation_time = 100;
time_step = 0.1;
N = round(Simulation_time / time_step);

v_des = 30;        % desired speed (m/s)
T = 1.5;           % time headway (s)
s0 = 4;            % minimum gap (m), a.k.a. s0 (your "gamma")
num_vehicles = 5;

% --- Initialize
vehicles = struct([]);
for i = 1:num_vehicles
    vehicles(i).id = i;
    vehicles(i).road = 1;

    % Preallocate trajectories
    vehicles(i).pos   = nan(1, N+1);
    vehicles(i).speed = nan(1, N+1);
    vehicles(i).acc   = nan(1, N+1);

    % Initial states
    vehicles(i).pos(1)   = (num_vehicles - i) * (40 + s0) + 4*rand(); % spaced back
    vehicles(i).speed(1) = 25 + (30-25)*rand();                      % 25–30 m/s
    vehicles(i).acc(1)   = 0;

    % IDM params (per vehicle)
    vehicles(i).a     = 2 + (5-2)*rand();     % max accel
    vehicles(i).b     = 1 + (4-1)*rand();     % comfortable decel
    vehicles(i).delta = 3 + (4-3)*rand();     % accel exponent (usually 4)
    vehicles(i).Length = 4 + (6-4)*rand();    % vehicle length (m)

    % Predecessor index (0 = no leader)
    vehicles(i).preceding_vehicle_index = i-1;
end
vehicles(1).preceding_vehicle_index = 0; % leader



% --- Simulation loop. This is the main loop where we update our vehicle
% trajerctories based on the IDM model. Play a little bit to see how each
% variable works. 

% As a next you need to define a trigger condition and based on available
% data train using BLR and predict. In the paper you can find some hint on
% how to select the number of data to train etc. But to make it
% simple you can say if a vehicle passes 60 meters in distance I train
% using data I have. Then we can do some comparisons how the amount of data
% can influence conservatism. 


for t = 1:N
    % Recompute order & predecessors each step (by position)
    [~, order] = sort(arrayfun(@(v) v.pos(t), vehicles), 'descend'); % larger x = ahead
    % Map: who is ahead of whom. We do that in order to easily identify the
    % preceding vehicle. We will do something similar when we have merging,
    % but it will not be the same. 
    inv_order = zeros(1,num_vehicles);
    inv_order(order) = 1:num_vehicles;
    for k = 1:num_vehicles
        rk = inv_order(k); % rank in sorted list
        if rk == 1
            vehicles(k).preceding_vehicle_index = 0; % no leader
        else
            leader_id = order(rk - 1);
            vehicles(k).preceding_vehicle_index = leader_id;
        end
    end

    % Update each vehicle
    for i = 1:num_vehicles
        
        v  = vehicles(i).speed(t);
        %the following 3 parameters are associated with some
        %characteristics of the vehicles (like maximum acceleration or
        %comfortable deceleration) and are used by the IDM model. The
        %parameter del does not have a physical interpretation. Look the
        %IDM model in google to see what this parameter does (it is not
        %important for now tho).
        a0 = vehicles(i).a;
        b0 = vehicles(i).b;
        del= vehicles(i).delta;


        %we check if there is a preceding vehicle. If it is not we need to
        %consider a special case of the IDM which is much simpler(see else)
        if vehicles(i).preceding_vehicle_index >= 1

    % we calculate some parameters needed for the IDM Model
            j = vehicles(i).preceding_vehicle_index;  % leader index
            vL = vehicles(j).speed(t);
            x  = vehicles(i).pos(t);
            xL = vehicles(j).pos(t);

            % actual net spacing s (subtract leader length)
            s  = xL - x - vehicles(j).Length;
            s  = max(s, 1e-3); % avoid divide-by-zero

            dv = v - vL; % positive if approaching
            s_star = s0 + v*T + (v * dv) / (2*sqrt(a0*b0));

            acc = a0 * ( 1 - (v / v_des)^del - (s_star / s)^2 );
        else
            % Leader uses free-road term only
            acc = a0 * ( 1 - (v / v_des)^del );
        end

        % we we update our vehicles state based on the acceleration we
        % obtain from the IDM model.
        vehicles(i).acc(t)     = acc;
        vehicles(i).speed(t+1) = max(0, v + time_step * acc);
        vehicles(i).pos(t+1)   = vehicles(i).pos(t) + time_step * v + 0.5 * time_step^2 * acc;
    end
end


% Here we plot!


figure(1);
hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).pos);
end

hold off

figure(2);

hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).speed);
end

hold off

figure(3);

hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).acc);
end

hold off

%% --- NEWELL MODEL & BLR ESTIMATION SECTION
disp("Starting BLR estimation using IDM trajectories...");

% 1. Extract position/speed data into matrices
t = 0:time_step:Simulation_time;
Tlen = numel(t);
num_vehicles = numel(vehicles);
P = cell2mat(arrayfun(@(v) v.pos(:), vehicles, 'uni', false)); % (T x N)
V = cell2mat(arrayfun(@(v) v.speed(:), vehicles, 'uni', false));

% 2. Build leader indices per timestep
leaders = zeros(Tlen, num_vehicles);
for ti = 1:Tlen
    pos_t = arrayfun(@(v) v.pos(ti), vehicles);
    [~, ord] = sort(pos_t, 'descend');           % larger x = ahead
    invord = zeros(1, num_vehicles);
    invord(ord) = 1:num_vehicles;
    for k = 1:num_vehicles
        rk = invord(k);                          % rank (1 = front)
        if rk == 1
            leaders(ti,k) = 0;                   % no leader
        else
            leaders(ti,k) = ord(rk - 1);         % leader = one car ahead
        end
    end
end


% 3. Estimate Newell time shifts τ_k(t)
w = 10;  % backward wave speed (m/s)
taus_all = nan(Tlen, num_vehicles);
for k = 1:num_vehicles
    j = leaders(:,k);

    if any(j > 0)
        % Case 1: has a real leader at some times
        % Use that leader's trajectory for tau estimation
        Pj = P(:, j(find(j>0,1)));  % pick first nonzero leader index
        taus_all(:,k) = estimate_tau_series(t, Pj, P(:,k), w);
    else
        % Case 2: no leader at all (first vehicle)
        % Use virtual constant-speed leader instead
        Fj_virtual = make_virtual_leader_interp(t, P(:,k), time_step);
        Pj_virtual = Fj_virtual(t);
        taus_all(:,k) = estimate_tau_series(t, Pj_virtual, P(:,k), w);
    end
end

% 4. Build dataset for BLR and train model
[X, Y] = make_blr_dataset(t, P, leaders, taus_all);
blr = blr_fit(X, Y);

% 5. Predict τ for one follower and plot
k = 3;
idx_ok = find(~isnan(taus_all(:,k)));
Xk = []; Yk = [];
for ii = idx_ok(:).'
    j = leaders(ii,k); if j==0, continue; end
    Xk(end+1,:) = [1, P(ii,k), P(ii,j)];
    Yk(end+1,1) = taus_all(ii,k);
end
[m_tau, v_tau] = blr_predict(blr, Xk);

figure;
plot(t(idx_ok), Yk, 'k', t(idx_ok), m_tau, 'r', ...
     t(idx_ok), m_tau + 2*sqrt(v_tau), 'r--', ...
     t(idx_ok), m_tau - 2*sqrt(v_tau), 'r--');
xlabel('time (s)'); ylabel('\tau (s)');
title('BLR prediction vs IDM-derived τ');
legend('True τ','BLR mean','±2σ band');

% --- Helper functions below ---
function taus = estimate_tau_series(t, Pj, Pk, w)
    Fj = griddedInterpolant(t, Pj, 'pchip');
    taus = nan(size(t)); guess=1.5;
    for ii=1:numel(t)
        fun = @(tau) Fj(t(ii)-tau) - w*tau - Pk(ii);
        try
            taus(ii)=fzero(fun, guess); guess=max(0.2,min(3,taus(ii)));
        catch, taus(ii)=NaN; end
    end
end

function [X,Y] = make_blr_dataset(t,P,leaders,taus)
    [T,N] = size(P); X=[]; Y=[];
    for k=2:N
        idx=find(~isnan(taus(:,k)));
        for i=idx(:).'
            j=leaders(i,k); if j==0, continue; end
            X(end+1,:)=[1, P(i,k), P(i,j)];
            Y(end+1,1)=taus(i,k);
        end
    end
end

function model = blr_fit(X,Y)
    [N,M]=size(X); alpha=1e-3; beta=1/var(Y); I=eye(M);
    for it=1:200
        S=inv(alpha*I+beta*(X.'*X)); mu=beta*S*(X.'*Y);
        gamma=sum(1-alpha*diag(S));
        alpha_new=gamma/(mu.'*mu); err=Y-X*mu;
        beta_new=(N-gamma)/(err.'*err);
        if max(abs([alpha_new-alpha,beta_new-beta]))<1e-6, break; end
        alpha=alpha_new; beta=beta_new;
    end
    model.mu=mu; model.S=S; model.alpha=alpha; model.beta=beta;
end

function [m,v] = blr_predict(model,Xstar)
    m = Xstar*model.mu;
    v = sum((Xstar*model.S).*Xstar,2) + 1/model.beta;
end

function Fj = make_virtual_leader_interp(t, Pk, dt)
% make_virtual_leader_interp:
% Creates a constant-speed "imaginary" leader trajectory

    % Estimate the follower's average recent speed (use last 1 s)
    win = max(2, round(1.0/dt));
    vbar = mean(diff(Pk(max(1,end-win+1):end))) / dt;

    % If speeds are noisy or constant, fallback to global mean
    if ~isfinite(vbar)
        vbar = max(0.1, (Pk(end) - Pk(1)) / (t(end) - t(1)));
    end

    % Set an initial spacing (e.g. 10 m) so the leader starts ahead
    gap0 = 10;  
    p0   = Pk(1) + gap0 - vbar * t(1);

    % Define the leader's position trajectory
    Pj = vbar * t + p0;

    % Return a smooth interpolant so p_j'(t - τ) can be evaluated
    Fj = griddedInterpolant(t, Pj, 'pchip');
end