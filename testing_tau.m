

% In the following code I try to simulate human driven vehicle so that we
% can play with Bayesian Linear regression and see how it works. Because
% here we do not use VR, we need to generate vehicle trajectories (mimicing
% humans) in a different way. To do that, we use a car-following model.
% This model is the IDM model (you can google it). Here We will do a simple
% example considering only a single road. Not a merging. Next we initialize
% some parameters for our simulation. 

Simulation_time = 100;
time_step = 0.1;
N = round(Simulation_time / time_step);

v_des = 30;        % desired speed (m/s)
T = 1.5;           % time headway (s)
s0 = 4;            % minimum gap (m), a.k.a. s0 (your "gamma")
num_vehicles = 5;

% --- Initialize
vehicles = struct([]);
for i = 1:num_vehicles
    vehicles(i).id = i;
    vehicles(i).road = 1;

    % Preallocate trajectories
    vehicles(i).pos   = nan(1, N+1);
    vehicles(i).speed = nan(1, N+1);
    vehicles(i).acc   = nan(1, N+1);

    % Initial states
    vehicles(i).pos(1)   = (num_vehicles - i) * (40 + s0) + 4*rand(); % spaced back
    vehicles(i).speed(1) = 25 + (30-25)*rand();                      % 25–30 m/s
    vehicles(i).acc(1)   = 0;

    % IDM params (per vehicle)
    vehicles(i).a     = 2 + (5-2)*rand();     % max accel
    vehicles(i).b     = 1 + (4-1)*rand();     % comfortable decel
    vehicles(i).delta = 3 + (4-3)*rand();     % accel exponent (usually 4)
    vehicles(i).Length = 4 + (6-4)*rand();    % vehicle length (m)

    % Predecessor index (0 = no leader)
    vehicles(i).preceding_vehicle_index = i-1;
end
vehicles(1).preceding_vehicle_index = 0; % leader



% --- Simulation loop. This is the main loop where we update our vehicle
% trajerctories based on the IDM model. Play a little bit to see how each
% variable works. 

% As a next you need to define a trigger condition and based on available
% data train using BLR and predict. In the paper you can find some hint on
% how to select the number of data to train etc. But to make it
% simple you can say if a vehicle passes 60 meters in distance I train
% using data I have. Then we can do some comparisons how the amount of data
% can influence conservatism. 


for t = 1:N
    % Recompute order & predecessors each step (by position)
    [~, order] = sort(arrayfun(@(v) v.pos(t), vehicles), 'descend'); % larger x = ahead
    % Map: who is ahead of whom. We do that in order to easily identify the
    % preceding vehicle. We will do something similar when we have merging,
    % but it will not be the same. 
    inv_order = zeros(1,num_vehicles);
    inv_order(order) = 1:num_vehicles;
    for k = 1:num_vehicles
        rk = inv_order(k); % rank in sorted list
        if rk == 1
            vehicles(k).preceding_vehicle_index = 0; % no leader
        else
            leader_id = order(rk - 1);
            vehicles(k).preceding_vehicle_index = leader_id;
        end
    end

    % Update each vehicle
    for i = 1:num_vehicles
        
        v  = vehicles(i).speed(t);
        %the following 3 parameters are associated with some
        %characteristics of the vehicles (like maximum acceleration or
        %comfortable deceleration) and are used by the IDM model. The
        %parameter del does not have a physical interpretation. Look the
        %IDM model in google to see what this parameter does (it is not
        %important for now tho).
        a0 = vehicles(i).a;
        b0 = vehicles(i).b;
        del= vehicles(i).delta;


        %we check if there is a preceding vehicle. If it is not we need to
        %consider a special case of the IDM which is much simpler(see else)
        if vehicles(i).preceding_vehicle_index >= 1

    % we calculate some parameters needed for the IDM Model
            j = vehicles(i).preceding_vehicle_index;  % leader index
            vL = vehicles(j).speed(t);
            x  = vehicles(i).pos(t);
            xL = vehicles(j).pos(t);

            % actual net spacing s (subtract leader length)
            s  = xL - x - vehicles(j).Length;
            s  = max(s, 1e-3); % avoid divide-by-zero

            dv = v - vL; % positive if approaching
            s_star = s0 + v*T + (v * dv) / (2*sqrt(a0*b0));

            acc = a0 * ( 1 - (v / v_des)^del - (s_star / s)^2 );
        else
            % Leader uses free-road term only
            acc = a0 * ( 1 - (v / v_des)^del );
        end

        % we we update our vehicles state based on the acceleration we
        % obtain from the IDM model.
        vehicles(i).acc(t)     = acc;
        vehicles(i).speed(t+1) = max(0, v + time_step * acc);
        vehicles(i).pos(t+1)   = vehicles(i).pos(t) + time_step * v + 0.5 * time_step^2 * acc;
    end
end

%estimate tau
vehicles_per_hour = 1500;
w = 10*vehicles_per_hour/3600;
fprintf('w = %.2f\n',w)

for i = 2:num_vehicles
    j = vehicles(i).preceding_vehicle_index;
    if j > 0
        fprintf('\n Vehicle %d following Vehicle %d:\n', i,j)
        tau_range = 0.1:0.01:10.0;
        errors = zeros(size(tau_range));

        for k = 1:length(tau_range)
            tau_test = tau_range(k);
            tau_steps = round(tau_test/time_step);

            if tau_steps < length(vehicles(j).pos)
                p_leader_shifted = vehicles(j).pos(1:end-tau_steps);
                p_follower_predicted = p_leader_shifted - w*tau_test;
                p_follower_actual = vehicles(i).pos(tau_steps+1:end);
                n_compare = min(length(p_follower_predicted),length(p_follower_actual));
                errors(k) = mean((p_follower_predicted(1:n_compare)-p_follower_actual(1:n_compare)).^2);
            else
                errors(k) = inf;
            end
        end

        [min_error,min_index] = min(errors);
        tau_best = tau_range(min_index);
    
        vehicles(i).newell_tau = tau_best;
        vehicles(i).newell_w = w;
        vehicles(i).newell_rmse = sqrt(min_error);
        vehicles(i).standstill_space = w*tau_best;
    

        fprintf('Estimated Tau = %.2f seconds\n',tau_best)
        fprintf('RMSE = %.2f meters\n',sqrt(min_error))
    end
end


% Here we plot!


figure(1);
hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).pos,'DisplayName', ['Vehicle ' num2str(i)]);
end
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Positions Over Time');
legend('show', 'Location', 'best');
grid on;
hold off

figure(2);

hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).speed,'DisplayName', ['Vehicle ' num2str(i)]);
end
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Vehicle Speeds Over Time');
legend('show', 'Location', 'best');
grid on;

hold off

figure(3);

hold on 

t=0:time_step:Simulation_time;

for i=1:num_vehicles
    plot(t,vehicles(i).acc,'DisplayName', ['Vehicle ' num2str(i)]);
end
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
title('Vehicle Accelerations Over Time');
legend('show', 'Location', 'best');
grid on;

hold off