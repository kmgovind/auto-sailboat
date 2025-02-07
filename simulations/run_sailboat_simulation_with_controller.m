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

% Desired heading
desired_heading = pi/2; % 90 degrees

% Time step for the simulation
dt = 0.1;

% Run the simulation with controller
[t, state] = ode45(@(t, state) sailboat_dynamics(t, state, params, simple_controller(desired_heading, state(3), wind.psi_tw, state(4), wind.a_tw, dt), wind, currents), time_span, initial_conditions);

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

% Extract rudder and sail commands over time
rudder_commands = zeros(length(t), 1);
sail_commands = zeros(length(t), 1);
for i = 1:length(t)
    commands = simple_controller(desired_heading, state(i, 3), wind.psi_tw, state(i, 4), wind.a_tw, dt);
    [rudder_commands(i), sail_commands(i)] = deal(commands.delta_r, commands.delta_s);
end

% Plot rudder commands over time
figure;
plot(t, rudder_commands);
title('Rudder Commands Over Time');
xlabel('Time (s)');
ylabel('Rudder Angle (rad)');
grid on;

% Plot sail commands over time
figure;
plot(t, sail_commands);
title('Sail Commands Over Time');
xlabel('Time (s)');
ylabel('Sail Angle (rad)');
grid on;

% Create an animation of the sailboat dynamics
figure;
filename = 'results/sailboat_simulation_w_control.gif';
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
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', dt);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt);
    end
end