clear; clc; close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
initial_conditions = [0; 0; 0; 1]; % Initial state [x; y; theta; v]
dt = 0.1; % Time step

% Define waypoints
waypoints = [10, 10; 20, 5; 30, 15; 40, 10];

% Define wind conditions
wind = struct('a_tw', 1.0, 'psi_tw', pi/4);

% Define dynamic obstacles
obstacles = [5, 5; 15, 10; 25, 20]; % Example obstacle positions

% Initialize state
state = initial_conditions;
t = 0;
trajectory = state';

waypoint_counter = 1;

% Run the simulation
for i = 1:dt:time_span(end)
    % Get the current waypoint
    waypoint = waypoints(waypoint_counter, :);

    % Compute the control input using the waypoint controller
    controls = struct('delta_r', dubins_waypoint_controller(state, waypoint));

    % Apply APF-based obstacle avoidance
    controls = apf_obstacle_avoidance(state, controls, obstacles);

    % Integrate the state using the Dubins boat model
    [~, state] = ode45(@(t, state) dubins_boat_model(t, state, controls, wind), [t t+dt], state);
    state = state(end, :)';

    % Append the new state to the trajectory
    trajectory = [trajectory; state'];

    % Check if the boat is within 0.1 meters of the waypoint
    if norm(state(1:2) - waypoint') < 0.1
        waypoint_counter = waypoint_counter + 1;
        % If all waypoints are reached, break the loop
        if waypoint_counter > size(waypoints, 1)
            break;
        end
    end

    % Update time
    t = t + dt;
end

results_dir = "results";
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Plot results
figure;
plot(trajectory(:, 1), trajectory(:, 2));
hold on;
scatter(waypoints(:, 1), waypoints(:, 2), 'r', 'filled');
scatter(obstacles(:, 1), obstacles(:, 2), 'k', 'filled');
legend('Trajectory', 'Waypoints', 'Obstacles');
title('Dubins Boat Trajectory with Obstacle Avoidance');
xlabel('X Position');
ylabel('Y Position');
grid on;
saveas(gcf, fullfile(results_dir, 'dubins_boat_trajectory_obstacle_avoidance.png'));

% Pad the heading vs time so that the last value is held
time_vector = 0:dt:time_span(end);
if length(time_vector) > size(trajectory, 1)
    padding_length = length(time_vector) - size(trajectory, 1);
    last_heading = trajectory(end, 3);
    padded_heading = [trajectory(:, 3); repmat(last_heading, padding_length, 1)];
else
    padded_heading = trajectory(:, 3);
end

figure;
plot(time_vector, padded_heading);
title('Dubins Boat Heading Over Time');
xlabel('Time (s)');
ylabel('Heading (rad)');
grid on;
saveas(gcf, fullfile(results_dir, 'dubins_boat_heading_obstacle_avoidance.png'));

% Waypoint-following controller
function delta_r = dubins_waypoint_controller(state, waypoint)
    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3);

    % Compute the desired heading
    desired_heading = atan2(waypoint(2) - y, waypoint(1) - x);

    % Compute the heading error
    heading_error = desired_heading - theta;

    % Normalize the heading error to the range [-pi, pi]
    heading_error = atan2(sin(heading_error), cos(heading_error));

    % Simple proportional controller for the rudder angle
    k_p = 1.0;
    delta_r = k_p * heading_error;
end

% APF-based obstacle avoidance
function controls = apf_obstacle_avoidance(state, controls, obstacles)
    % Parameters for APF
    k_repulsive = 10;
    k_attractive = 1;
    repulsive_radius = 5;

    % Initialize the force vector
    force = [0, 0];

    % Attractive force towards the waypoint
    waypoint = [state(1) + cos(state(3)), state(2) + sin(state(3))];
    force = force + k_attractive * (waypoint - state(1:2)');

    % Repulsive force from obstacles
    for j = 1:size(obstacles, 1)
        obstacle = obstacles(j, :);
        distance = norm(state(1:2)' - obstacle);
        if distance < repulsive_radius
            force = force - k_repulsive * (1 / distance - 1 / repulsive_radius) * (state(1:2)' - obstacle) / distance^3;
        end
    end

    % Adjust the control inputs based on the force
    controls.delta_r = controls.delta_r + atan2(force(2), force(1));
end

% Dubins boat model
function dstate = dubins_boat_model(t, state, controls, wind)
    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);

    % Control inputs
    delta_r = controls.delta_r;

    % Wind effects
    a_tw = wind.a_tw;
    psi_tw = wind.psi_tw;

    % Equations of motion
    dx = v * cos(theta) + a_tw * cos(psi_tw);
    dy = v * sin(theta) + a_tw * sin(psi_tw);
    dtheta = v / 2 * tan(delta_r); % Simple model for rudder effect
    dv = 0; % Constant speed for simplicity

    % Return the state derivatives
    dstate = [dx; dy; dtheta; dv];
end
