function acc = idm_dvdt(v_a, s_a, dv_a, p)
    % IDM acceleration for follower

    v0   = p.v0;
    T    = p.T;
    s0   = p.s0;
    a    = p.a;
    b    = p.b;
    if isfield(p,'delta')
        delta = p.delta;
    else
        delta = 4;
    end

    % clamp to safe ranges
    v_a = max(v_a, 0);
    s_a = max(s_a, 1e-3);
    v0  = max(v0, 1);
    T   = max(T, 0.1);
    s0  = max(s0, 0);
    a   = max(a, 0.1);
    b   = max(b, 0.1);

    % free-road term
    phi_free = (v_a / v0)^delta;

    % desired dynamic gap
    den_ab = max(a*b, 1e-4);
    s_star = s0 + v_a*T + (v_a*dv_a) / (2*sqrt(den_ab));
    s_star = max(s_star, 0);

    % interaction
    phi_int = (s_star / s_a)^2;
    phi_int = min(phi_int, 1e4);  % cap

    % IDM accel
    acc = a * (1 - phi_free - phi_int);
end
