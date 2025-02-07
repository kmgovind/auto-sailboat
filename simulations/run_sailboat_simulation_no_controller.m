% Ready workspace
clear;clc;close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
initial_conditions = [0; 0; 0; 10; 0]; % Initial state [x; y; theta; v; omega]

% Get model parameters
params = define_params();

% Define ocean currents
currents = struct('v_cx', 0.5, 'v_cy', 1.3);

% Define initial wind conditions
initial_wind_speed = 5.0;  % m/s
initial_wind_direction = 45;  % degrees

% Define time vector for wind simulation
wind_time = linspace(time_span(1), time_span(2), length(time_span)*10); % Higher resolution for gusts

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

% Fixed control inputs
controls = struct('delta_s', 0, 'delta_r', 0); % Fixed sail and rudder angles

% Run the simulation without controller
[t, state] = ode45(@(t, state) sailboat_dynamics(t, state, params, controls, wind, currents), time_span, initial_conditions);

% Plot results
figure;
plot(state(:, 1), state(:, 2));
title('Sailboat Trajectory');
xlabel('X Position');
ylabel('Y Position');
grid on;

figure;
plot(t, state(:, 3));
title('Sailboat Heading Over Time');
xlabel('Time (s)');
ylabel('Heading (rad)');
grid on;

% Quiver plot for ocean currents
x_min = min(state(:, 1));
x_max = max(state(:, 1));
y_min = min(state(:, 2));
y_max = max(state(:, 2));
[X, Y] = meshgrid(linspace(x_min, x_max, 20), linspace(y_min, y_max, 20));
U = currents.v_cx * ones(size(X));
V = currents.v_cy * ones(size(Y));
figure;
quiver(X, Y, U, V);
hold on;
plot(state(:, 1), state(:, 2), 'r', 'LineWidth', 2);
title('Ocean Currents and Sailboat Trajectory');
xlabel('X Position');
ylabel('Y Position');
grid on;

% Plot wind speed over time
figure;
plot(wind_time, wind_speeds, 'b', 'LineWidth', 2);
title('Wind Speed Over Time');
xlabel('Time (s)');
ylabel('Wind Speed (m/s)');
grid on;

% Plot wind direction over time
figure;
plot(wind_time, rad2deg(wind_directions), 'r', 'LineWidth', 2);
title('Wind Direction Over Time');
xlabel('Time (s)');
ylabel('Wind Direction (degrees)');
grid on;

% Create an animation of the sailboat dynamics
figure;

% Define the results directory 
results_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
filename = fullfile(results_dir, 'sailboat_simulation_no_control.gif');

for i = 1:length(t)
    % Clear figure
    clf;
    
    % Plot sailboat trajectory
    plot(state(1:i, 1), state(1:i, 2), 'b', 'LineWidth', 2);
    hold on;

    % Plot sailboat heading (red)
    quiver(state(i, 1), state(i, 2), cos(state(i, 3)), sin(state(i, 3)), 'r', 'LineWidth', 2);

    % Plot ocean current vector (green)
    quiver(state(i, 1), state(i, 2), currents.v_cx, currents.v_cy, 'g', 'LineWidth', 2);

    % Interpolate wind at current time step
    wind_speed_now = interp1(wind.t, wind.a_tw, t(i), 'linear', 'extrap');
    wind_direction_now = interp1(wind.t, wind.psi_tw, t(i), 'linear', 'extrap');
    
    % Compute wind vector components
    wind_x = wind_speed_now * cos(wind_direction_now);
    wind_y = wind_speed_now * sin(wind_direction_now);
    
    % Plot wind vector (black)
    quiver(state(i, 1), state(i, 2), wind_x, wind_y, 'k', 'LineWidth', 2); 

    hold off;

    % Set plot title and labels
    title('Sailboat Dynamics with Wind and Currents');
    xlabel('X Position');
    ylabel('Y Position');
    grid on;
    axis([x_min x_max y_min y_max]);

    % **Dynamically update the legend with actual values**
    legend_text = {
        sprintf('Sailboat Trajectory (%.2f m/s)', state(i, 4)), ...
        sprintf('Sailboat Heading: %.2f°', rad2deg(state(i, 3))), ...
        sprintf('Ocean Current: %.2fm/s @ %.2f°', norm([currents.v_cx, currents.v_cy]), rad2deg(atan2(currents.v_cy, currents.v_cx))), ...
        sprintf('Wind: %.2fm/s @ %.2f°', wind_speed_now, rad2deg(wind_direction_now))
    };

    legend(legend_text, 'Location', 'best');

    drawnow;
    
    % Capture the plot as an image
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write to the GIF File
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
    end
end