function [xL_traj, vL_traj] = rollout_leader_idm(x0_L, pL, N, dt)
% rollout_leader_idm
% Roll out a leader using IDM and no front car (i.e. free flow)
% x0_L : [xL0; vL0]
% pL   : struct('v0','T','s0','a','b','delta')
% N    : horizon
% dt   : sampling time

xL = x0_L(1);
vL = x0_L(2);

xL_traj = zeros(N,1);
vL_traj = zeros(N,1);

for k = 1:N
    % here we assume leader is in free flow (no front car), so s* term is 0
    % reuse safe IDM
    accL = idm_dvdt_safe(vL, 1e6, 0, pL);   % huge gap -> free road

    vL = max(0, vL + dt*accL);
    xL = xL + dt*vL + 0.5*(dt^2)*accL;

    xL_traj(k) = xL;
    vL_traj(k) = vL;
end
end
