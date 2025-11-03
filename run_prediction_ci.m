% run_prediction_ci.m
% =========================================================
% 1) roll out leader from *given* IDM params
% 2) simulate follower using your identified idnlgrey model m
% 3) propagate your parameter covariance to follower trajectory
% 4) plot nominal + 95% CI
% =========================================================

% clear; clc; close all;

%% 1. settings
dt = 0.02;
N  = 1000;                 % 4 seconds
t  = (0:N-1)' * dt;

%% 2. LEADER: given (mean) IDM parameters
leader_params.v0   = 15;   % m/s
leader_params.T    = 1.5;  % s
leader_params.s0   = 2.0;  % m
leader_params.a    = 1.0;  % m/s^2
leader_params.b    = 1.5;  % m/s^2
leader_params.delta= 4;

xL0  = 15;    % leader starts 15 m ahead of follower
vL0  = 15;    % leader initial speed
x0_L = [xL0; vL0];

% you can also make the leader follow some accel input; here pure IDM w.r.t. imaginary car
[xL_traj, vL_traj] = rollout_leader_idm(x0_L, leader_params, N, dt);

% leader input to follower (what follower senses):
u_future = [xL_traj, vL_traj];   % [N x 2]

%% 3. FOLLOWER: load / have your identified idnlgrey model
% IMPORTANT: m must be in workspace; here we just assume you have it
% e.g. you did before:
%   load('identified_idm_model.mat','m')
% or you ran your parameter_estimation and got m there.
%
% For the sake of completeness, I will just assume m exists:
% (remove this error once you have m)
if ~exist('m','var')
    error('You must have your identified idnlgrey model ''m'' in the workspace.');
end

% initial follower state (you can take it from data; here, example)
xE0 = 0;
vE0 = 12;
x0_follower = [xE0; vE0];

%% 4. do prediction + CI using *your* covariance
[t, y_nom, ci_low, ci_high] = follower_ci_from_idnlgrey( ...
    m, u_future, x0_follower, N, dt);

%% 5. plot
figure; hold on;
% CI band
% --- Confidence Interval (make it stand out) ---
fill([t; flipud(t)], [ci_high; flipud(ci_low)], [0 0.2 0.8], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.65);  % Deep navy blue, solid fill
% nominal follower
plot(t, y_nom, 'b', 'LineWidth', 2);
% leader (for reference)
plot(t, xL_traj, 'r--', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Position [m]');
legend('95% CI (follower)','Follower nominal','Leader','Location','Best');
title('Follower prediction with parameter-uncertainty CI');
grid on;
