clear
data = load('preceding_p.mat');


% --- Constants ---
dt = 0.02;
w = 10;               % backward wave speed (m/s)
tau_bar = 1.5;        % nominal delay (s)
g0 = 50;              % headway offset (m)
Stimulation_Time = 400;
N_full = round(Stimulation_Time / dt) + 1;

P_true = data.P;

P_true = -P_true;
K = load("preceding_k.mat", 'K').K;           % number of observed data points

% === Run parameter estimation ===
[P_pred, pL, mu_tau_k, sigma2_p_k] = parameter_estimation(P_true, K, dt, w, tau_bar, g0, Stimulation_Time);


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



function [P_pred, pL, mu_tau_k, sigma2_p_k] = parameter_estimation(P_true, K, dt, w, tau_bar, g0, Stimulation_Time)
% PARAMETER_ESTIMATION
% - Coordinates: positions INCREASE downstream (x forward).
% - Newell:  pk(t) = pL(t - tau) - w*tau,  w >= 0
% - Leader:  constant-speed virtual leader  pL(t) = phi1*t + phi0
% - BLR for tau: inputs x = [1, pk, pL],  tau ~ N(X*theta, beta^-1),  theta ~ N(0, alpha^-1 I)
%
% OUTPUTS:
%   P_pred     = stitched trajectory: true data up to K, then predicted mean after K
%   pL         = virtual leader trajectory on t_full
%   mu_tau_k   = BLR mean of tau at the stitch time K   (local τ̂K)
%   sigma2_p_k = propagated position variance band after K (constant over t for Lemma 2)

    % --- Hyperparams (can auto-tune via evidence; kept explicit here) ---
    alpha = 2.0;   % prior precision on weights
    beta  = 10.0;  % noise precision on tau observations (std ~ 0.316 s)

    % --- Time vectors ---
    N_full    = round(Stimulation_Time/dt) + 1;
    t_obs     = (0:dt:(K-1)*dt)';         % observed times
    t_full    = (0:dt:Stimulation_Time)'; % full horizon
    pk_full   = P_true(1:K)';             % follower positions (observed)

    % --- Virtual leader (constant speed) ---
    % Positions increase: leader "ahead" => larger position by g0 at t0.
    [phi0, phi1, pL] = make_virtual_leader(t_full, t_obs, pk_full, tau_bar, g0);

    % --- Interpolant for pL(t) at arbitrary times (needed for Newell root) ---
    F_lead = griddedInterpolant(t_full, pL, 'pchip');

    % =========================
    % STEP 1: Extract samplewise tau_i by solving Newell at each ti
    % =========================
    tau_obs_all = nan(K-1, 1);
    guess_tau   = tau_bar;  % warm start

    for i = 1:(K-1)
        ti    = t_obs(i);
        pk_i  = pk_full(i);
        % Newell residual for positions (increasing coordinate):
        % f(tau) = pL(ti - tau) - w*tau - pk(ti) = 0
        f = @(tau) F_lead(ti - tau) - w*tau - pk_i;

        try
            % Try warm-start; if it fails, try a safe bracket [0, 6] s
            tau_i = fzero(f, guess_tau);
        catch
            try
                tau_i = fzero(f, [0, 6]);
            catch
                tau_i = NaN;
            end
        end

        if tau_i < 0, tau_i = NaN; end
        tau_obs_all(i) = tau_i;
        if isfinite(tau_i), guess_tau = tau_i; end
    end

    valid = isfinite(tau_obs_all);
    tau_obs = tau_obs_all(valid);

    % Align p_k and p_L with those same time indices (first K-1 samples)
    pk_used = pk_full(valid);
    % t_obs aligns with the first K-1 entries of t_full, so pL(valid) is OK:
    pj_used = pL(1:K-1);
    pj_used = pj_used(valid);

    % =========================
    % STEP 2: Fit BLR for tau = theta0 + theta1*pk + theta2*pj
    % =========================
    X = [ones(numel(pk_used),1), pk_used(:), pj_used(:)];
    Y = tau_obs(:);

    % Posterior over weights:  Sigma = (alpha I + beta X'X)^(-1),  mu = beta Sigma X'Y
    A = alpha*eye(3) + beta*(X'*X);
    Sigma_theta = A \ eye(3);          % inv(A) but numerically stabler
    mu_theta    = (A \ (beta*(X'*Y)));

    % =========================
    % STEP 3: Use LOCAL τ at the stitch time K (not a global average)
    % =========================
    % Paper uses τ's local mean at the time of interest. At the handoff (tK),
    % evaluate BLR at xK = [1, p_k(K), p_L(K)].
    tK   = t_obs(K);                 % stitch time
    xK   = [1, P_true(K), pL(K)];    % features at K
    mu_tau_k = xK * mu_theta;        % local mean τ̂K
    % Predictive variance for τ at K (optional, for bands):
    sigma2_tau_k = xK * Sigma_theta * xK.' + 1/beta;

    % =========================
    % STEP 4: Propagate via Lemma 2 (leader is affine), using τ̂K
    % =========================
    % From Newell with affine leader: mu_p_k(t) = phi1*t + (phi0 - (phi1 + w)*mu_tau_k)
    mu_p_k = phi1 .* t_full + (phi0 - (phi1 + w) * mu_tau_k);

    % Variance band from Lemma 2 (constant over t for this case):
    sigma2_p_k = ((phi1 + w)^2) * sigma2_tau_k * ones(size(t_full));

    % =========================
    % STEP 5: Stitch: data up to K, prediction after K (no ad-hoc offset)
    % =========================
    P_pred = nan(N_full,1);
    P_pred(1:K)      = P_true(1:K);
    P_pred(K+1:end)  = mu_p_k(K+1:end);

end

% ===== Helper: constant-speed virtual leader with time shift tau_bar =====
function [phi0, phi1, pL] = make_virtual_leader(t_full, t_obs, pf, tau_bar, g0)
    % Estimate leader speed from a short window near the stitch for robustness
    win = max(20, round(2 / (t_obs(2)-t_obs(1))));         % ~2 s window
    i1  = max(1, numel(t_obs)-win);
    phi1 = (pf(end) - pf(i1)) / (t_obs(end) - t_obs(i1)); % constant speed

    % Leader starts g0 meters AHEAD at the first observation time t0
    t0 = t_obs(1);
    p0 = pf(1) + g0;                  % positions increase => ahead = larger

    % Choose phi0 so that pL(t0 - tau_bar) = p0  (time-lead alignment)
    phi0 = p0 - phi1 * (t0 - tau_bar);

    % Leader trajectory on the whole grid
    pL = phi1 * t_full + phi0;
end