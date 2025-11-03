function [xnext, y] = idm_dt_nl(~, x, u, ...
                                v0, T_norm, s0, a_norm, b, delta_norm, Ts, scale_y, varargin)
    % States: x = [xE; vE]
    % Inputs: u = [xL; vL]
    % We estimate normalized params and map to real IDM here.

    % --- decode normalized parameters ---
    T_nominal     = 2.0;   % seconds
    a_nominal     = 2.0;   % m/s^2
    delta_nominal = 4.0;   % -
    T     = T_nominal     * T_norm;
    a     = a_nominal     * a_norm;
    delta = delta_nominal * delta_norm;

    % --- unpack states, inputs ---
    xE = x(1);
    vE = x(2);
    xL = u(1);
    vL = u(2);

    llead = 3;
    s  = xL - xE - llead;
    dv = vE - vL;

    % IDM params struct
    p = struct('v0',v0, 'T',T, 's0',s0, 'a',a, 'b',b, 'delta',delta);

    % safe gap
    s_eff = max(s, 1e-3);

    % acceleration
    acc = idm_dvdt(vE, s_eff, dv, p);

    % discrete update
    vE2 = max(0, vE + Ts*acc);
    xE2 = xE + Ts*vE + 0.5*(Ts^2)*acc;

    xnext = [xE2; vE2];
 %% THIS IS THE PLACE WHERE I RESCALE THE OUTPUT TO MATCH THE SCALING WE DID PREVIOUSLY
    % output (already scaled, since data was scaled)
    y = xE2 / scale_y;
 %% END THIS IS THE PLACE WHERE I RESCALE THE OUTPUT TO MATCH THE SCALING WE DID PREVIOUSLY


end
