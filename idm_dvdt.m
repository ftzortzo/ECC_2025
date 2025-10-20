function dv = idm_dvdt(v_a, s_a, dv_a, p)
% IDM acceleration dot v_alpha for follower alpha
% Inputs:
% v_a   : v_alpha  (speed, m/s)
% s_a   : s_alpha  (bumper-to-bumper gap to leader, m)
% dv_a  : Delta v_alpha = v_alpha - v_{lead}  (approaching rate, m/s)
% p     : struct with fields v0, T, s0, a, b, delta
% Output:
% dv    : dot v_alpha  (acceleration, m/s^2)

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
    dv = a * (1 - phi_free - phi_int);
end