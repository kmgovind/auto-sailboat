% Ready workspace
clear;clc;close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
initial_conditions = [0; 0; 0; 0; 0]; % Initial state [x; y; theta; v; omega]

% Get model parameters
params = define_params();

% Define ocean currents
currents = struct('v_cx', 0.5, 'v_cy', 1.3);

% Define wind conditions
wind = struct('a_tw', 2.0, 'psi_tw', pi/4);

% Fixed control inputs
% controls = struct('delta_s', pi/6, 'delta_r', pi/12); % Fixed sail and rudder angles
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


% Create an animation of the sailboat dynamics
figure;
filename = 'results/sailboat_simulation_no_control.gif';
for i = 1:length(t)
    plot(state(1:i, 1), state(1:i, 2), 'b', 'LineWidth', 2);
    hold on;
    quiver(state(i, 1), state(i, 2), cos(state(i, 3)), sin(state(i, 3)), 'r', 'LineWidth', 2);
    quiver(state(i, 1), state(i, 2), currents.v_cx, currents.v_cy, 'g', 'LineWidth', 2);
    hold off;
    title('Sailboat Dynamics');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Sailboat Trajectory', 'Sailboat Heading', 'Ocean Current', 'Location', 'best');
    grid on;
    axis([x_min x_max y_min y_max]);
    drawnow;
    
    % Capture the plot as an image
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write to the GIF File
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.25);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.25);
    end
end