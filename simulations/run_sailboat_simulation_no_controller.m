% Ready workspace
clear;clc;close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
initial_conditions = [0; 0; 0; 0; 0]; % Initial state [x; y; theta; v; omega]

% Define model parameters
params = struct('p1', 0.1, 'p2', 0.01, 'p3', 0.1, 'p4', 0.5, 'p5', 0.2, ...
                'p6', 0.5, 'p7', 0.3, 'p8', 0.2, 'p9', 1.5, 'p10', 0.5, ...
                'p11', 0.1);

% Define ocean currents
currents = struct('v_cx', 0.5, 'v_cy', 1.3);

% Define wind conditions
wind = struct('a_tw', 2.0, 'psi_tw', pi/4);

% Fixed control inputs
controls = struct('delta_s', pi/6, 'delta_r', pi/12); % Fixed sail and rudder angles

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