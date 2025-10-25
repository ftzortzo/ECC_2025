data = load('P_true_data.mat');
% --- Constants ---
test = 1;
dt = 0.02;
w = 10;               % backward wave speed (m/s)
tau_bar = 1.5;        % nominal delay (s)
g0 = 10;              % headway offset (m)
Stimulation_Time = 5000;
N_full = round(Stimulation_Time / dt) + 1;

K = 804
[P_pred, pL, test] = parameter_estimation(data.P_true, K);

function [P_pred, pL, test] = parameter_estimation(P_true, K)


    % --- Constants ---
    test = 1;
    dt = 0.02;
    w = 10;               % backward wave speed (m/s)
    tau_bar = 1.5;        % nominal delay (s)
    g0 = 10;              % headway offset (m)
    Stimulation_Time = 5000;
    N_full = round(Stimulation_Time / dt) + 1;

    % --- Observed data ---
    t_obs = (0:dt:(K-1)*dt)';      % observation times
    t_full = (0:dt:Stimulation_Time)
    pf = P_true(1:K)';             % observed trajectory (distance to merge)
   
    % --- Detect direction (increase or decrease over time) ---
    if pf(end) < pf(1)
        dir = -1;  % distance decreasing over time (typical)
    else
        dir = 1;   % distance increasing (position-type data)
    end

    % --- Virtual leader trajectory ---
    pL = make_virtual_leader(t_full, pf, tau_bar, g0, dir);

    % --- Initialize predicted trajectory ---
    P_pred = nan(N_full, 1);
    P_pred(1:K) = pf;    % copy observed data up to K

    % --- Create smooth leader interpolant ---
    F_lead = griddedInterpolant(t_full, pL, 'pchip');

    % --- Prediction loop using Newell's relation ---
    tau_est = tau_bar;
    for i = K+1:N_full
        t_curr = (i-1)*dt;
        t_query = t_curr - tau_est;

        if t_query < t_obs(1)
            P_pred(i) = pf(end);    % fallback to last known follower pos
        else
            % Newell model: follower position based on delayed leader
            P_pred(i) = F_lead(t_query) - dir * w * tau_est;
        end
    end

    % --- Debug output ---
    % if test
    %     disp('Stimulation: Parameter Estimation Summary');
    %     fprintf('Observed samples K = %.2f\n', K);
    %     fprintf('pf(1)=%.2f, pf(end)=%.2f, pL(1)=%.2f, pL(K+100)=%.2f\n, pL(K+200)=%.2f\n', ...
    %         pf(1), pf(end), pL(1), pL(K+100), pL(K+200));
    %     fprintf('Pred(K)=%.2f, Pred(K+100)=%.2f, Pred(K+200)=%.2f\n', ...
    %         P_pred(K), P_pred(min(K+100, end)), P_pred(K+200));
    % end
end


% --- Helper: Virtual leader trajectory (Eq. 31–33 analog) ---
function pL = make_virtual_leader(t_full, pf, tau_bar, g0, dir)
    % Average slope (velocity magnitude)
    t_obs = linspace(t_full(1), t_full(length(pf)), length(pf));
    phi1 = ((pf(end) - pf(1)) / (t_obs(end) - t_obs(1)));
    phi1 = phi1(1);  

    % Apply direction (negative slope if decreasing distances)
    phi1 = dir * abs(phi1);
    
    % Starting conditions
    t0 = t_obs(1);
    p0 = pf(1) + dir * g0;   % leader starts ahead by g0

    % Linear trajectory shifted by tau_bar
    phi0 = p0 - phi1 * (t0 - tau_bar);
    pL = phi1 * t_full(:) + phi0;
end

%% === Continuous trajectory visualization ===
figure; hold on;

% --- Time vectors ---
t_obs  = (0:dt:(K-1)*dt)';              % observed time (true data)
t_full = (0:dt:(Stimulation_Time))'; % full simulation horo

% --- Plot actual observed trajectory (deep black) ---
plot(t_obs, P_true(1:K), 'W', 'LineWidth', 1.8);

% --- Plot virtual leader (blue, dotted) ---
plot(t_full, pL, 'b--', 'LineWidth', 1.4);

% --- Plot predicted trajectory (lighter red, dashed) ---
plot(t_full, P_pred, 'r--', 'LineWidth', 1.6);

% --- Optional: connect smoothly at handoff ---
plot(t_obs(end), P_true(K), 'ko', 'MarkerFaceColor', 'k');
text(t_obs(end)+0.3, P_true(K), 'handoff →', 'Color', 'k', 'FontSize', 10);

% --- Labels & aesthetics ---
xlabel('Time (s)');
ylabel('Distance to merging point (m)');
title('Follower Distance Prediction using Virtual Leader (Newell Model)');
legend({'Observed (True)', 'Virtual Leader', 'Predicted (Newell)'}, ...
       'Location', 'northeast');
xlim([0, min(35, t_full(end))]);
ylim([0, max(P_pred(1), 1000)]);
grid on;
hold off;
