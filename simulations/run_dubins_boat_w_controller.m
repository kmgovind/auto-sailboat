clear;clc;close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
dt = 0.1; % Time step size
initial_conditions = [0; 0; 0; 1]; % Initial state [x; y; theta; v]
N = length(time_span(1):dt:time_span(2)); % Number of time steps

% Define waypoints
waypoints = [10, 10; 20, 5; 30, 15; 40, 10; 0, 0];

% Define wind conditions
% wind = struct('a_tw', 1.0, 'psi_tw', pi/4);

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
    angle_offset_rad = deg2rad(20 + (45 - 20) * rand);

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

% Initialize state
state = initial_conditions;
t = 0;
dt = 0.1; % Time step
trajectory = state';

waypoint_counter = 1;

% Run the simulation
k = 1;
for i = 1:dt:time_span(end)
    % Get the current waypoint
    waypoint = waypoints(waypoint_counter, :);

    % Compute the control input using the waypoint controller
    % controls = struct('delta_r', dubins_waypoint_controller(state, waypoint));
    controls = struct('delta_r', lqr_controller(state, waypoint, currents.v_cx(k), currents.v_cy(k)));
    k = k + 1;
    % Integrate the state using the Dubins boat model
    [~, state] = ode45(@(t, state) dubins_boat_model(t, state, controls, wind, currents), [t t+dt], state);
    state = state(end, :)';

    % Append the new state to the trajectory
    trajectory = [trajectory; state'];

    % Check if the boat is within 1 meters of the waypoint
    if norm(state(1:2) - waypoint') < 0.5 %threshold
        waypoint_counter = waypoint_counter + 1;
        % If all waypoints are reached, break the loop
        if waypoint_counter > size(waypoints, 1)
            break;
        end
    end

    % Update time
    t = t + dt;
end

results_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

% Plot results

figure;
plot(trajectory(:, 1), trajectory(:, 2));
hold on;
scatter(waypoints(:, 1), waypoints(:, 2), 'r', 'filled');
legend('Trajectory', 'Waypoints', 'Location', 'southeast');
title('Dubins Boat Trajectory');
xlabel('X Position');
ylabel('Y Position');
grid on;
saveas(gcf, fullfile(results_dir, 'dubins_boat_trajectory.png'));

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
saveas(gcf, fullfile(results_dir, 'dubins_boat_heading.png'));

% plot wind and currents
figure;
subplot(2, 1, 1);
plot(wind.t, wind.a_tw);
title('Wind Speed Over Time');
xlabel('Time (s)');
ylabel('Wind Speed (m/s)');
grid on;
saveas(gcf, fullfile(results_dir, 'wind_speed.png'));
subplot(2, 1, 2);
plot(currents.t, currents.v_cx);
title('Current X Component Over Time');
xlabel('Time (s)');
ylabel('Current Speed (m/s)');
grid on;

% Create a simple animation of the boat's trajectory with wind/current arrows
make_gif = true; 

% Scaling factors for boat and wind arrows
boat_arrow_scale = 1;   % scales boat velocity arrow
wind_arrow_scale = 0.3;   % scales wind arrow

if make_gif
    fig_anim = figure;
    gif_name = fullfile(results_dir, 'boat_animation.gif');

    for idx = 1:size(trajectory, 1)
        clf;
        hold on; grid on;
        
        % Plot path so far
        plot(trajectory(1:idx,1), trajectory(1:idx,2), 'b', 'LineWidth', 1.5);
        
        % Plot waypoints
        plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerFaceColor', 'r');

        % Scale boat's heading arrow by its velocity (stored in trajectory(:,4))
        quiver(trajectory(idx,1), trajectory(idx,2), ...
            boat_arrow_scale * trajectory(idx,4) * cos(trajectory(idx,3)), ...
            boat_arrow_scale * trajectory(idx,4) * sin(trajectory(idx,3)), ...
            0, 'r', 'LineWidth', 2);

        % Interpolate and scale wind vector
        t_now = time_vector(min(idx, length(time_vector)));
        w_speed = interp1(wind.t, wind.a_tw, t_now, 'linear', 'extrap');
        w_dir   = interp1(wind.t, wind.psi_tw, t_now, 'linear', 'extrap');
        quiver(trajectory(idx,1), trajectory(idx,2), ...
            wind_arrow_scale * w_speed * cos(w_dir), ...
            wind_arrow_scale * w_speed * sin(w_dir), ...
            0, 'k', 'LineWidth', 2);

        % Interpolate and plot current vector (unchanged)
        c_x = interp1(currents.t, currents.v_cx, t_now, 'linear', 'extrap');
        c_y = interp1(currents.t, currents.v_cy, t_now, 'linear', 'extrap');
        quiver(trajectory(idx,1), trajectory(idx,2), ...
            c_x, c_y, ...
            0, 'g', 'LineWidth', 2);

        axis equal;
        axis([0 45 0 20]); % Adjust as needed
        title('Boat Trajectory with Wind and Current');
        xlabel('X Position'); ylabel('Y Position');
        drawnow;

        % Capture frame
        frame_data = getframe(fig_anim);
        [im_ind, cm] = rgb2ind(frame_data.cdata, 256);
        if idx == 1
            imwrite(im_ind, cm, gif_name, 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
        else
            imwrite(im_ind, cm, gif_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
        end
    end
end


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