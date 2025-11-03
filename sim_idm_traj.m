function y = sim_idm_traj(m, x0, u_seq, N)
% Simulates your idnlgrey IDM model for N steps.
% m     : idnlgrey
% x0    : initial follower state [xE; vE]
% u_seq : [N x 2] leader signals
% N     : horizon length
%
% returns y: Nx1 model output (scaled position in your setup)

x = x0(:);
y = zeros(N,1);

% turn model name into callable function
f = str2func(m.FileName);

for k = 1:N
    u_k = u_seq(k, :).';

    % call your model file, e.g. idm_dt_nl(...)
    [x_next, y_k] = f([], x, u_k, ...
        m.Parameters(1).Value, ...  % v0
        m.Parameters(2).Value, ...  % T_norm
        m.Parameters(3).Value, ...  % s0
        m.Parameters(4).Value, ...  % a_norm
        m.Parameters(5).Value, ...  % b
        m.Parameters(6).Value, ...  % delta_norm
        m.Parameters(7).Value, ...  % Ts
        m.Parameters(8).Value);     % scale_y

    x = x_next;
    y(k) = y_k;
end
end