data = load('P_true_data.mat');

% --- Constants ---
dt = 0.02;
w = 10;               % backward wave speed (m/s)
tau_bar = 1.5;        % nominal delay (s)
g0 = 80;              % headway offset (m)
Stimulation_Time = 1000;
N_full = round(Stimulation_Time / dt) + 1;

P_true = data.P_true;
K = 803;              % number of observed data points

% === Run parameter estimation ===
[P_pred, pL, sigma2_p_k, ignored_1, ignored_2] = parameter_estimation(P_true, K, dt, w, tau_bar, g0, Stimulation_Time);

%% === Continuous trajectory visualization ===
figure; hold on;

t_obs  = (0:dt:(K-1)*dt)';      % observed time (true data)
t_full = (0:dt:Stimulation_Time)'; % full time horizon
sigma_p_k = sqrt(sigma2_p_k);
P_high = P_pred + 1.96 * sigma_p_k;   % upper 95%
P_low  = P_pred - 1.96 * sigma_p_k;   % lower 95%

% --- Plot confidence interval bounds (light red lines) ---
plot(t_full, P_high, 'r:', 'LineWidth', 1.0);
plot(t_full, P_low,  'r:', 'LineWidth', 1.0);

fill([t_full; flipud(t_full)], [P_high; flipud(P_low)], ...
     [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% --- Plot actual observed trajectory (deep black) ---
plot(t_obs, P_true(1:K), 'k', 'LineWidth', 1.8);

% --- Plot virtual leader (blue, dotted) ---
plot(t_full, pL, 'b--', 'LineWidth', 1.4);

% --- Plot predicted trajectory (lighter red, dashed) ---
plot(t_full, P_pred, 'r--', 'LineWidth', 1.6);

% --- Optional: connect smoothly at handoff ---
plot(t_obs(end), P_true(K), 'ko', 'MarkerFaceColor', 'k');
text(t_obs(end)+0.3, P_true(K), 'handoff →', 'Color', 'k', 'FontSize', 10);

xlabel('Time (s)');
ylabel('Distance to merging point (m)');
title('Follower Distance Prediction using BLR-based τ_k (Lemma 2)');
legend({'Observed (True)', 'Virtual Leader', 'Predicted (BLR + Lemma 2)'}, ...
       'Location', 'northeast');
xlim([0, min(35, t_full(end))]);
ylim([0, 1000]);
grid on;
hold off;



%% === Main Estimation Function ===
function [P_pred, pL, sigma2_p_k, phi_k1, phi_k0] = parameter_estimation(P_true, K, dt, w, tau_bar, g0, Stimulation_Time)
    % === Constants ===
    alpha = 2.0;   % BLR prior precision
    beta  = 10.0;  % BLR noise precision
    N_full = round(Stimulation_Time / dt) + 1;

    % === Observed data ===
    t_obs_full = (0:dt:(K-1)*dt)';   
    pk_full = P_true(1:K)';           

    % === Detect direction ===
    dir = -1;
    if pk_full(end) >= pk_full(1)
        dir = 1;
    end

    % === Time vectors ===
    t_full = (0:dt:Stimulation_Time)'; 

    % === Virtual leader ===
    [phi_j0, phi_j1, pL] = make_virtual_leader(t_full, t_obs_full, pk_full, tau_bar, g0, dir);

    % === Interpolation ===
    F_lead_local = griddedInterpolant(t_full, pL, 'pchip');

    % === Step 1: Compute tau_obs (Eq. 29 solver) ===
    disp("Estimating τ_k (time shift) from observations...");
    tau_obs_all = nan(K-1, 1);
    guess_tau = tau_bar;

    for i = 1:(K-1)
        tk = t_obs_full(i);
        pk_val = pk_full(i);

        % f(τ) = p_j(t_i - τ) - wτ - p_k(t_i) = 0
        f = @(tau) F_lead_local(tk - tau) - w*tau - pk_val;

        try
            tau_i = fzero(f, guess_tau);
            if tau_i < 0, tau_i = NaN; end
            tau_obs_all(i) = tau_i;
            guess_tau = tau_i;
        catch
            tau_obs_all(i) = NaN;
        end
    end

    valid_idx = isfinite(tau_obs_all);
    tau_obs = tau_obs_all(valid_idx);
    pk_used = pk_full(valid_idx);
    pj_used = pL(valid_idx);
    
% === Step 2: BLR fitting ===
X = [ones(numel(pk_used), 1), pk_used(:), pj_used(:)];
Y = tau_obs(:);

I3 = eye(3);
Sigma_theta = inv(beta * (X' * X) + alpha * I3);
mu_theta = beta * Sigma_theta * (X' * Y);

disp("BLR fitted:"); 
disp(mu_theta.'); 
disp(Sigma_theta.');

% --- Time-varying tau_k(t) prediction (Eq. 30) ---
tau_blr_pred = X * mu_theta;  % τ̂_k(t) = θ0 + θ1*p_k + θ2*p_j
xK = [1, P_true(K), pL(K)];
mu_tau_k = xK * mu_theta;    % local τ̂K

% === Lemma 2 (Eq. 41, time-varying μτₖ(t)) ===
% Compute μ_p_k(t) using τ̂_k(t)
t_obs_used = t_obs_full(valid_idx);
mu_p_k = phi_j1 .* t_full + (phi_j0 - (phi_j1 + w) * mu_tau_k);


% Approximate σ²_p_k from variance of τ̂
sigma2_tau_k = var(tau_blr_pred, 'omitnan');
sigma2_p_k = (phi_j1 + w).^2 .* sigma2_tau_k .* ones(size(t_full));

% === Final predicted trajectory ===
P_pred = nan(N_full, 1);
P_pred(1:K) = P_true(1:K);
P_pred(K+1:end) = mu_p_k(K+1:end);

phi_k1 = phi_j1;
phi_k0 = phi_j0 - (phi_j1 + w) * mu_tau_k;
end



%% --- Helper: Virtual leader trajectory ---
function [phi0, phi1, pL] = make_virtual_leader(t_full, t_obs, pf, tau_bar, g0, dir)
    % Compute slope (velocity magnitude)
    
    phi1 = (pf(end) - pf(1)) / (t_obs(end) - t_obs(1));
    phi1 = dir * abs(phi1);

    % Initial conditions
    t0 = t_obs(1);
    p0 = pf(1) + dir * g0;  % leader starts ahead by g0

    % Linear trajectory shifted by tau_bar
    phi0 = p0 - phi1 * (t0 - tau_bar);
    pL = phi1 * t_full + phi0;
end
