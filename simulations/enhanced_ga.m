% filepath: c:\Users\kavin\OneDrive\School\Winter_2025\ROB_599\auto-sailboat\simulations\enhanced_test_ga.m
% Genetic Algorithm for Sailboat Control Optimization

clear; clc; close all;

% Simulation parameters
time_span = [0 1000]; % Total simulation time
dt = 0.1; % Time step
initial_conditions = [0; 0; 0]; % [x; y; theta]
N = round(diff(time_span) / dt); % Number of steps
waypoints = [10 10; 20 5; -30 50; 40 10; -50 20; 100 0; 150 15];
goal_tolerance = 2; % How close is "close enough" to switch waypoints

% Genetic Algorithm parameters
population_size = 100; % Number of boats in each generation
num_generations = 100; % Number of generations
mutation_rate = 0.65; % Probability of mutation
elite_fraction = 0.2; % Fraction of top performers to carry over

% Initialize population (each individual is [Kp, no_go_angle])
population = [rand(population_size, 1) * 5, deg2rad(rand(population_size, 1) * 30)];

% Define wind simulation
initial_wind_speed = 1.5;
initial_wind_direction = 30; % deg
wind_time = linspace(time_span(1), time_span(2), N);
wind_speeds = zeros(size(wind_time));
wind_directions = zeros(size(wind_time));

for i = 1:N
    wind_instance = enhanced_wind_model(wind_time(i), initial_wind_speed, initial_wind_direction);
    wind_speeds(i) = wind_instance.speed;
    wind_directions(i) = wind_instance.direction;
end

wind = struct('t', wind_time, 'a_tw', wind_speeds, 'psi_tw', deg2rad(wind_directions));

% Currents based on wind
currents = struct('t', wind_time, 'v_cx', zeros(1, N), 'v_cy', zeros(1, N));
for i = 1:N
    angle_offset_rad = deg2rad(20 + (45 - 20) * rand);
    speed_ratio = 0.01 + (0.03 - 0.01) * rand;
    current_speed = speed_ratio * wind_speeds(i);
    current_direction = deg2rad(wind_directions(i)) - angle_offset_rad;
    currents.v_cx(i) = current_speed * cos(current_direction);
    currents.v_cy(i) = current_speed * sin(current_direction);
end

% Genetic Algorithm loop
for generation = 1:num_generations
    fitness = zeros(population_size, 1);
    
    % Evaluate each individual in the population
    for i = 1:population_size
        params = population(i, :);
        fitness(i) = simulate_boat(params, waypoints, wind, currents, initial_conditions, dt, N, goal_tolerance);
    end
    
    % Select top performers (elitism)
    [~, sorted_indices] = sort(fitness, 'descend');
    num_elite = round(elite_fraction * population_size);
    new_population = population(sorted_indices(1:num_elite), :);
    
    % Generate offspring through crossover and mutation
    while size(new_population, 1) < population_size
        % Select two parents
        parents = population(sorted_indices(randi(num_elite, 2, 1)), :);
        % Crossover
        child = mean(parents, 1);
        % Mutation
        if rand < mutation_rate
            child = child + [randn * 0.5, deg2rad(randn * 5)];
        end
        new_population = [new_population; child];
    end
    
    % Update population
    population = new_population(1:population_size, :);
    
    % Display progress
    disp(['Generation ', num2str(generation), ': Best fitness = ', num2str(max(fitness))]);
end


% Run the simulation with the best controller
best_params = population(sorted_indices(1), :);
x = initial_conditions(1);
y = initial_conditions(2);
theta = initial_conditions(3);
waypoint_index = 1;
goal = waypoints(waypoint_index, :)';
path = zeros(N, 3); % [x, y, theta]
control_inputs = zeros(N, 1); % [u]

for k = 1:N
    t = (k-1)*dt;
    
    % Interpolate wind and current
    wind_speed = interp1(wind.t, wind.a_tw, t);
    wind_dir = interp1(wind.t, wind.psi_tw, t);
    wind_vector = wind_speed * [cos(wind_dir); sin(wind_dir)];
    current = [currents.v_cx(k); currents.v_cy(k)];
    
    % Check proximity to goal and switch to next waypoint
    dist_to_goal = norm([x; y] - goal);
    if dist_to_goal < goal_tolerance
        waypoint_index = waypoint_index + 1;
        if waypoint_index > size(waypoints, 1)
            break;
        end
        goal = waypoints(waypoint_index, :)';
    end
    
    % Controller
    u = sailboat_controller_v2(x, y, theta, goal(1), goal(2), wind_vector, best_params);
    
    % Boat speed model
    rel_wind_dir = wrapToPi(atan2(wind_vector(2), wind_vector(1)) - theta);
    beta = abs(rel_wind_dir);
    boat_speed = max(0, wind_speed * cos(beta) * 0.5);
    
    % Update state (Euler)
    x = x + dt * (boat_speed * cos(theta) + current(1));
    y = y + dt * (boat_speed * sin(theta) + current(2));
    theta = wrapToPi(theta + dt * u);
    
    % Store path and control inputs
    path(k, :) = [x, y, theta];
    control_inputs(k) = u;
end

% Trim unused rows
path = path(1:k, :);
control_inputs = control_inputs(1:k);

% Save and plot results
% save('simulation_results.mat', 'path', 'control_inputs', 'dt', 'waypoints');

figure;
subplot(2, 1, 1);
plot(path(:, 1), path(:, 2), '-b', 'LineWidth', 1.5);
hold on;
plot(waypoints(:, 1), waypoints(:, 2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
xlabel('X Position');
ylabel('Y Position');
title('Path of the Sailboat');
grid on;
legend('Path', 'Waypoints');

subplot(2, 1, 2);
time = (0:length(control_inputs)-1) * dt;
plot(time, control_inputs, '-r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (u)');
title('Control Inputs vs Time');
grid on;

% Save the best controller parameters and simulation results
save('controller_and_results.mat', 'best_params', 'path', 'control_inputs', 'dt', 'waypoints');
disp('Controller parameters and simulation results saved to controller_and_results.mat');

% --- Simulation Function ---
function fitness = simulate_boat(params, waypoints, wind, currents, initial_conditions, dt, N, goal_tolerance)
    x = initial_conditions(1);
    y = initial_conditions(2);
    theta = initial_conditions(3);
    waypoint_index = 1;
    goal = waypoints(waypoint_index, :)';
    fitness = 0;
    
    for k = 1:N
        t = (k-1)*dt;
        
        % Interpolate wind and current
        wind_speed = interp1(wind.t, wind.a_tw, t);
        wind_dir = interp1(wind.t, wind.psi_tw, t);
        wind_vector = wind_speed * [cos(wind_dir); sin(wind_dir)];
        current = [currents.v_cx(k); currents.v_cy(k)];
        
        % Check proximity to goal and switch to next waypoint
        dist_to_goal = norm([x; y] - goal);
        if dist_to_goal < goal_tolerance
            waypoint_index = waypoint_index + 1;
            if waypoint_index > size(waypoints, 1)
                fitness = fitness + 1000; % Bonus for completing all waypoints
                break;
            end
            goal = waypoints(waypoint_index, :)';
        end
        
        % Controller
        u = sailboat_controller_v2(x, y, theta, goal(1), goal(2), wind_vector, params);
        
        % Boat speed model
        rel_wind_dir = wrapToPi(atan2(wind_vector(2), wind_vector(1)) - theta);
        beta = abs(rel_wind_dir);
        boat_speed = max(0, wind_speed * cos(beta) * 0.5);
        
        % Update state (Euler)
        x = x + dt * (boat_speed * cos(theta) + current(1));
        y = y + dt * (boat_speed * sin(theta) + current(2));
        theta = wrapToPi(theta + dt * u);
        
        % Accumulate fitness (e.g., minimize distance to goal)
        fitness = fitness - dist_to_goal;
    end
end

% --- Controller Function ---
function u = sailboat_controller_v2(x, y, theta, x_goal, y_goal, wind, params)
    Kp = params(1);
    no_go_angle = params(2);
    
    goal_vec = [x_goal - x; y_goal - y];
    theta_goal = atan2(goal_vec(2), goal_vec(1));
    wind_dir = atan2(wind(2), wind(1));
    
    angle_to_wind = wrapToPi(theta_goal - wind_dir);
    
    if abs(angle_to_wind) < no_go_angle
        tack_sign = sign(sin(theta - wind_dir));
        theta_desired = wrapToPi(wind_dir + tack_sign * no_go_angle * 1.3);
    else
        theta_desired = theta_goal;
    end
    
    e_theta = wrapToPi(theta_desired - theta);
    u = Kp * e_theta;
end

% --- Wind Model Stub ---
function wind = enhanced_wind_model(t, base_speed, base_dir_deg)
    wind.speed = base_speed + 0.5 * sin(0.1 * t + 2 * pi * rand);
    wind.direction = base_dir_deg + 10 * sin(0.05 * t + 2 * pi * rand);
end