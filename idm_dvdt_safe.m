function acc = idm_dvdt_safe(v_a, s_a, dv_a, p)
% safe IDM acceleration

    v0   = max(p.v0, 1);
    T    = max(p.T,  0.1);
    s0   = max(p.s0, 0);
    a    = max(p.a,  0.1);
    b    = max(p.b,  0.1);
    delta= p.delta;

    % free road
    phi_free = (v_a / v0)^delta;

    % desired gap
    den_ab = max(a*b, 1e-4);
    s_star = s0 + v_a*T + (v_a*dv_a)/(2*sqrt(den_ab));
    s_star = max(s_star, 0);

    % interaction
    s_a = max(s_a, 1e-3);
    phi_int = (s_star / s_a)^2;
    phi_int = min(phi_int, 1e4);

    acc = a * (1 - phi_free - phi_int);
end
