clear; clc; close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
dt = 0.1; % Time step size
t = time_span(1):dt:time_span(2); % Time vector
N = length(t); % Number of time steps

% Initial state [x; y; theta; v; omega]
initial_conditions = [0; 0; 0; 1; 0];

% Define waypoint
waypoint = [10, 10];

% Get model parameters
params = define_params();

% Define initial wind conditions
initial_wind_speed = 10.0;  % m/s
initial_wind_direction = 45;  % degrees

% Define time vector for wind simulation
wind_time = linspace(time_span(1), time_span(2), N);

% Simulate wind conditions over time
wind_speeds = zeros(size(wind_time));
wind_directions = zeros(size(wind_time));

for i = 1:length(wind_time)
    wind_instance = enhanced_wind_model(wind_time(i), initial_wind_speed, initial_wind_direction);
    wind_speeds(i) = wind_instance.speed;
    wind_directions(i) = wind_instance.direction;
end

% Store wind structure for interpolation
wind = struct('t', wind_time, 'a_tw', wind_speeds, 'psi_tw', deg2rad(wind_directions));

% % Currents will be 20-45 degrees to the right of the wind, and only 1-3% of the wind's speed, direction and speed randomly change within these parameters
currents = struct('t', wind_time, 'v_cx', zeros(1,N), 'v_cy', zeros(1,N));

for i = 1:N
    % Random angle offset [20, 45] (degrees)
    angle_offset_deg = 20 + (45 - 20) * rand;
    angle_offset_rad = deg2rad(angle_offset_deg);

    % Random speed ratio [0.01, 0.03]
    speed_ratio = 0.01 + (0.03 - 0.01) * rand;

    % Current speed is (speed_ratio * wind speed)
    current_speed = speed_ratio * wind_speeds(i);

    % Current direction is wind_direction + angle_offset
    current_direction = deg2rad(wind_directions(i)) + angle_offset_rad;

    % Convert to Cartesian components
    currents.v_cx(i) = current_speed * cos(current_direction);
    currents.v_cy(i) = current_speed * sin(current_direction);
end

% Initialize state and control variables
state = zeros(N, 5);
state(1, :) = initial_conditions';
delta_r = zeros(N, 1);
delta_s = zeros(N, 1);

% Run the simulation with the lower level controller
for k = 1:N-1
    % Current state
    x = state(k, 1);
    y = state(k, 2);
    theta_b = state(k, 3);
    v_b = state(k, 4);
    omega_b = state(k, 5);

    % Compute control inputs using the lower level controller
    [delta_r(k), delta_s(k)] = lower_level_control(x, y, theta_b, v_b, waypoint(1), waypoint(2), wind.a_tw(k), wind.psi_tw(k), currents.v_cx(k), currents.v_cy(k));

    % Define control inputs
    controls = struct('delta_s', delta_s(k), 'delta_r', delta_r(k));

    % Compute the next state using the sailboat dynamics
    dstate = sailboat_dynamics(t(k), state(k, :)', params, controls, wind, currents);
    state(k+1, :) = state(k, :) + dt * dstate';

    % Normalize theta_b to be within [0, 2*pi]
    state(k+1, 3) = mod(state(k+1, 3), 2*pi);

    % Check if the sailboat is within 5 meters of the waypoint
    distance_to_waypoint = sqrt((x - waypoint(1))^2 + (y - waypoint(2))^2);
    if distance_to_waypoint <= 5
        fprintf('Reached within 5 meters of the waypoint at time %.2f seconds.\n', t(k));
        state = state(1:k+1, :);
        delta_r = delta_r(1:k);
        delta_s = delta_s(1:k);
        t = t(1:k+1);
        break;
    end
end

% Convert angles from radians to degrees for plotting
delta_r_deg = rad2deg(delta_r);
delta_s_deg = rad2deg(delta_s);

% Plot results
figure;
plot(state(:, 1), state(:, 2), 'b-', 'LineWidth', 2);
hold on;
plot(initial_conditions(1), initial_conditions(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(waypoint(1), waypoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('Sailboat Trajectory to Waypoint');
xlabel('X Position');
ylabel('Y Position');
legend('Trajectory', 'Start', 'Waypoint');
grid on;

figure;
plot(t, rad2deg(state(:, 3)));
title('Sailboat Heading Over Time');
xlabel('Time (s)');
ylabel('Heading (deg)');
grid on;

figure;
plot(t, state(:, 4));
title('Sailboat Speed Over Time');
xlabel('Time (s)');
ylabel('Speed (m/s)');
grid on;

figure;
plot(t, delta_r_deg);
title('Rudder Angle Over Time');
xlabel('Time (s)');
ylabel('Rudder Angle (deg)');
grid on;

figure;
plot(t, delta_s_deg);
title('Sail Angle Over Time');
xlabel('Time (s)');
ylabel('Sail Angle (deg)');
grid on;

% plot wind and current
figure;
subplot(2, 1, 1);
plot(wind.t, wind.a_tw, 'b-', 'LineWidth', 2);
hold on;
plot(wind.t, wind.psi_tw, 'r-', 'LineWidth', 2);
title('Wind Speed and Direction Over Time');
xlabel('Time (s)');
ylabel('Wind Speed (m/s) / Direction (rad)');
legend('Wind Speed', 'Wind Direction');
grid on;    

subplot(2, 1, 2);
plot(currents.t, currents.v_cx, 'b-', 'LineWidth', 2);
hold on;
plot(currents.t, currents.v_cy, 'r-', 'LineWidth', 2);
title('Current Speed Over Time');
xlabel('Time (s)');
ylabel('Current Speed (m/s)');
legend('Current X Component', 'Current Y Component');
grid on;

%current and wind on one graph with current scale on right
figure;
yyaxis left;
plot(wind.t, wind.a_tw, 'b-', 'LineWidth', 2);
hold on;
plot(wind.t, wind.psi_tw, 'r-', 'LineWidth', 2);
ylabel('Wind Speed (m/s) / Direction (rad)');
yyaxis right;
plot(currents.t, currents.v_cx, 'g-', 'LineWidth', 2);
hold on;
plot(currents.t, currents.v_cy, 'm-', 'LineWidth', 2);
ylabel('Current Speed (m/s)');
title('Wind and Current Speed Over Time');
xlabel('Time (s)');
legend('Wind Speed', 'Wind Direction', 'Current X Component', 'Current Y Component');
grid on;
hold off;
% End of simulation