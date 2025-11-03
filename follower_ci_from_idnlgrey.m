function [t, y_nom, ci_low, ci_high] = follower_ci_from_idnlgrey(m, u_future, x0, N, dt)
% Propagate follower's parameter covariance (from nlgreyest) to its future trajectory.
%
% m        : idnlgrey model (already identified)
% u_future : [N x 2] = [xL, vL] future leader signals (deterministic)
% x0       : 2x1 follower initial state
% N, dt    : horizon and sampling time

    % --- 1) parameter vector & covariance from nlgreyest ---
    theta_hat  = [m.Parameters.Value].';   % e.g. 8x1
    Sigma_full = getcov(m);                % 8x8

    % only params 2,4,6 are free: T_norm, a_norm, delta_norm
    idx = [2, 4, 6];
    Sigma_theta = Sigma_full(idx, idx);    % 3x3

    % scale for unscaling (you fixed scale_y)
    scale_y = theta_hat(8);

    % --- 2) nominal simulation (with estimated params) ---
    y_nom = sim_idm_traj(m, x0, u_future, N);   % scaled output

    % --- 3) finite-difference sensitivities ---
    p = numel(idx);        % 3
    S = zeros(N, p);
    eps_rel = 1e-4;

    for j = 1:p
        theta_pert = theta_hat;
        base_val   = theta_hat(idx(j));
        h          = eps_rel * max(1, abs(base_val));
        theta_pert(idx(j)) = base_val + h;

        % --- build perturbed model with CLIPPING to parameter bounds ---
        m_pert = m;
        for k = 1:numel(m_pert.Parameters)
            % original bounds
            pmin = m_pert.Parameters(k).Minimum;
            pmax = m_pert.Parameters(k).Maximum;

            % candidate value
            vnew = theta_pert(k);

            % clip to [min, max]
            vnew = min(max(vnew, pmin), pmax);

            m_pert.Parameters(k).Value = vnew;
        end

        % simulate perturbed model
        y_pert = sim_idm_traj(m_pert, x0, u_future, N);

        % sensitivity
        S(:, j) = (y_pert - y_nom) / h;
    end

    % --- 4) variance at each time step ---
    var_y = zeros(N,1);
    for k = 1:N
        s_k = S(k, :);                    % 1x3
        var_y(k) = s_k * Sigma_theta * s_k.';  % scalar
    end
    sigma_y = sqrt(max(var_y, 0));

    % --- 5) 95% CI in scaled units ---
    z95 = 1.96;
    ci_low_s  = y_nom - z95 * sigma_y;
    ci_high_s = y_nom + z95 * sigma_y;

    % --- 6) unscale back to meters ---
    y_nom   = y_nom   * scale_y;
    ci_low  = ci_low_s  * scale_y;
    ci_high = ci_high_s * scale_y;

    % --- 7) time grid ---
    t = (0:N-1).' * dt;
end
