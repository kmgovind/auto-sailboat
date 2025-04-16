% Simulate a sailboat using heading-based control under wind and current

clear; clc; close all;

% Define simulation parameters
time_span = [0 10000]; % Total simulation time
dt = 0.1; % Time step
initial_conditions = [0; 0; 0]; % [x; y; theta]
N = round(diff(time_span) / dt); % Number of steps

% Define waypoints (end at initial point to return)
% waypoints = [10 10; 20 5; 30 15; 40 10];
% waypoints = [

%         %  0  -100

%         %  0     0

%         -100 -400

%          -50 -600

%         -100 -200

%         -150  100

%     ];
waypoints = [10 10; 20 5; -30 50; 40 10; -50 20; 100 0; 150 15];
waypoint_index = 1;
goal = waypoints(waypoint_index, :)';

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
currents = struct('t', wind_time, 'v_cx', zeros(1,N), 'v_cy', zeros(1,N));
for i = 1:N
    angle_offset_rad = deg2rad(20 + (45 - 20) * rand);
    speed_ratio = 0.01 + (0.03 - 0.01) * rand;
    current_speed = speed_ratio * wind_speeds(i);
    current_direction = deg2rad(wind_directions(i)) - angle_offset_rad;
    currents.v_cx(i) = current_speed * cos(current_direction);
    currents.v_cy(i) = current_speed * sin(current_direction);
end

% Initialize state
x = initial_conditions(1);
y = initial_conditions(2);
theta = initial_conditions(3);
state_hist = zeros(3, N); % [x; y; theta]

% Main simulation loop
goal_tolerance = 2; % How close is "close enough" to switch waypoints
max_attempt_time = 250; % Maximum time (in seconds) to attempt reaching a waypoint
attempt_start_time = 0; % Track time spent attempting the current waypoint

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
        % Successfully reached the waypoint
        waypoint_index = waypoint_index + 1;
        if waypoint_index > size(waypoints, 1)
            state_hist = state_hist(:, 1:k-1);
            disp('All waypoints reached.');
            break;
        end
        goal = waypoints(waypoint_index, :)';
        disp(['Switching to next waypoint: ', num2str(goal')]);
        attempt_start_time = t; % Reset attempt time for the new waypoint
    elseif t - attempt_start_time > max_attempt_time
        % Skip the waypoint if it is unreachable
        disp(['Skipping unreachable waypoint: ', num2str(goal')]);
        waypoint_index = waypoint_index + 1;
        if waypoint_index > size(waypoints, 1)
            state_hist = state_hist(:, 1:k-1);
            disp('All waypoints reached.');
            break;
        end
        goal = waypoints(waypoint_index, :)';
        disp(['Switching to next waypoint: ', num2str(goal')]);
        attempt_start_time = t; % Reset attempt time for the new waypoint
    end
    
    % Controller
    u = sailboat_controller_v2(x, y, theta, goal(1), goal(2), wind_vector);
    
    % Boat speed model (simple function of wind angle to heading)
    rel_wind_dir = wrapToPi(atan2(wind_vector(2), wind_vector(1)) - theta);
    beta = abs(rel_wind_dir);
    boat_speed = max(0, wind_speed * cos(beta) * 0.5); % 0.5 is max efficiency
    
    % Update state (Euler)
    x = x + dt * (boat_speed * cos(theta) + current(1));
    y = y + dt * (boat_speed * sin(theta) + current(2));
    theta = wrapToPi(theta + dt * u);
    
    % Store state
    state_hist(:, k) = [x; y; theta];
end

% Define results directory
results_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Plot and save trajectory
figure;
plot(state_hist(1,:), state_hist(2,:), 'b-', 'LineWidth', 2); hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro--', 'LineWidth', 1.5);
plot(state_hist(1,1), state_hist(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('Sailboat Path through Waypoints');
xlabel('X'); ylabel('Y'); grid on; axis equal;
legend('Trajectory', 'Waypoints', 'Start');
saveas(gcf, fullfile(results_dir, 'trajectory.png'));

% Plot and save wind and currents
figure;
subplot(2, 1, 1);
plot(wind.t, wind.a_tw);
title('Wind Speed Over Time');
xlabel('Time (s)');
ylabel('Wind Speed (m/s)');
grid on;
subplot(2, 1, 2);
plot(currents.t, currents.v_cx);
title('Current X Component Over Time');
xlabel('Time (s)');
ylabel('Current Speed (m/s)');
grid on;
saveas(gcf, fullfile(results_dir, 'wind_and_currents.png'));

% % Animate the boat path and save as GIF
% figure;
% hold on;
% plot(waypoints(:,1), waypoints(:,2), 'ro--', 'LineWidth', 1.5);
% plot(state_hist(1,1), state_hist(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% trajectory_line = plot(state_hist(1,1), state_hist(2,1), 'b-', 'LineWidth', 2);
% boat_marker = plot(state_hist(1,1), state_hist(2,1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
% title('Sailboat Path Animation');
% xlabel('X'); ylabel('Y'); grid on; axis equal;
% legend('Waypoints', 'Start', 'Trajectory', 'Boat');

% % Initialize GIF
% gif_filename = fullfile(results_dir, 'sailboat_animation.gif');
% frame = getframe(gcf);
% im = frame2im(frame);
% [imind, cm] = rgb2ind(im, 256);
% imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);

% % Update animation and save frames to GIF
% for k = 1:10:length(state_hist(1,:))
%     set(trajectory_line, 'XData', state_hist(1,1:k), 'YData', state_hist(2,1:k));
%     set(boat_marker, 'XData', state_hist(1,k), 'YData', state_hist(2,k));
%     pause(0.1);
    
%     % Capture frame and append to GIF
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [imind, cm] = rgb2ind(im, 256);
%     imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
% end

% --- Controller Function ---
function u = sailboat_controller_v2(x, y, theta, x_goal, y_goal, wind)
    Kp = 3;
    no_go_angle = deg2rad(5);
    
    goal_vec = [x_goal - x; y_goal - y];
    theta_goal = atan2(goal_vec(2), goal_vec(1));
    wind_dir = atan2(wind(2), wind(1));
    
    angle_to_wind = wrapToPi(theta_goal - wind_dir);
    
    if abs(angle_to_wind) < no_go_angle
        % disp('Tacking!');
        % disp("Goal: " + [x_goal, y_goal]);
        % disp("Wind: " + wind);
        tack_sign = sign(sin(theta - wind_dir));
        theta_desired = wrapToPi(wind_dir + tack_sign * no_go_angle * 1.3);
    else
        theta_desired = theta_goal;
    end
    
    e_theta = wrapToPi(theta_desired - theta);
    u = Kp * e_theta;
end

% % --- Wind Model Stub ---
% function wind = enhanced_wind_model(t, base_speed, base_dir_deg)
%     % Add some sinusoidal variation for realism
%     wind.speed = base_speed + 0.5*sin(0.1*t + 2*pi*rand);
%     wind.direction = base_dir_deg + 10*sin(0.05*t + 2*pi*rand);
% end


