clear; clc; close all;

%% Simulation and NMPC Setup
% Simulation parameters
time_span = [0 100];       % Simulation time span in seconds
dt = 0.1;                  % Time step size
t = time_span(1):dt:time_span(2);  % Time vector
N_sim = length(t);         % Number of simulation time steps

% NMPC prediction horizon and dimensions
N_horizon = 20;            % NMPC prediction horizon (number of steps)
nx = 5;                    % Number of states [x; y; theta; v; omega]
nu = 2;                    % Number of control inputs [delta_r; delta_s]

% Initial state [x; y; theta; v; omega]
initial_conditions = [0; 0; 0; -1; 0];

% Define waypoint (desired [x, y])
waypoint = [10, 10];

% Get model parameters
params = define_params();

% Define ocean currents (assumed constant)
currents = struct('v_cx', 0.5, 'v_cy', 1.3);

% Define initial wind conditions (speed in m/s, direction in degrees)
initial_wind_speed = 5.0;  
initial_wind_direction = 45;

% Time vector for wind simulation and preallocation
wind_time = linspace(time_span(1), time_span(2), N_sim);
wind_speeds = zeros(size(wind_time));
wind_directions = zeros(size(wind_time));

for i = 1:length(wind_time)
    wind_instance = enhanced_wind_model(wind_time(i), initial_wind_speed, initial_wind_direction);
    wind_speeds(i) = wind_instance.speed;
    wind_directions(i) = wind_instance.direction;
end

% Store wind information (angles converted to radians)
wind = struct('t', wind_time, 'a_tw', wind_speeds, 'psi_tw', deg2rad(wind_directions));

% Preallocate state and control histories
state = zeros(N_sim, nx);
state(1,:) = initial_conditions';
delta_r_hist = zeros(N_sim, 1);
delta_s_hist = zeros(N_sim, 1);

% NMPC options (using fmincon with SQP)
options = optimoptions('fmincon','Display','none','Algorithm','sqp');

% Define control input bounds (example: rudder angle in [-30, 30]° and sail angle in [0, 90]°)
% Convert degrees to radians.
lb = repmat([ -deg2rad(30); 0 ], N_horizon, 1);
ub = repmat([  deg2rad(30); deg2rad(90) ], N_horizon, 1);

%% Main Simulation Loop with NMPC
for k = 1:N_sim-1
    % Current state
    current_state = state(k,:)';
    
    % For NMPC prediction, assume wind and currents remain constant over the horizon.
    current_wind.a_tw = wind.a_tw(k);
    current_wind.psi_tw = wind.psi_tw(k);
    
    % Set up the NMPC optimization problem as a direct shooting method.
    % Decision variable U_vec = [delta_r(0); delta_s(0); delta_r(1); delta_s(1); ... ]
    % The cost function penalizes the distance from the waypoint and the control effort.
    cost_fun = @(U_vec) nmpc_cost(U_vec, current_state, waypoint, params, current_wind, currents, dt, N_horizon);
    
    % Initial guess for U (zeros)
    U0 = zeros(N_horizon*nu, 1);
    
    % Solve the NMPC optimization problem using fmincon
    U_opt = fmincon(cost_fun, U0, [], [], [], [], lb, ub, [], options);
    
    % Extract the first control input from the optimal control sequence
    delta_r_opt = U_opt(1);
    delta_s_opt = U_opt(2);
    delta_r_hist(k) = delta_r_opt;
    delta_s_hist(k) = delta_s_opt;
    
    % Define control structure for dynamics
    controls = struct('delta_r', delta_r_opt, 'delta_s', delta_s_opt);
    
    % Compute next state using the sailboat dynamics function
    dstate = sailboat_dynamics_old(t(k), current_state, params, controls, wind, currents);
    state(k+1,:) = state(k,:) + dt * dstate';
    
    % Normalize heading angle theta to [0, 2*pi]
    state(k+1,3) = mod(state(k+1,3), 2*pi);
    
    % Check if the sailboat is within 5 meters of the waypoint
    distance_to_waypoint = norm(state(k,1:2)' - waypoint');
    if distance_to_waypoint <= 5
        fprintf('Reached within 5 meters of the waypoint at time %.2f seconds.\n', t(k));
        state = state(1:k+1,:);
        delta_r_hist = delta_r_hist(1:k);
        delta_s_hist = delta_s_hist(1:k);
        t = t(1:k+1);
        break;
    end
end

%% Plotting the Results

% Convert angles from radians to degrees for plotting
delta_r_deg = rad2deg(delta_r_hist);
delta_s_deg = rad2deg(delta_s_hist);

figure;
plot(state(:, 1), state(:, 2), 'b-', 'LineWidth', 2);
hold on;
plot(initial_conditions(1), initial_conditions(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(waypoint(1), waypoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('Sailboat Trajectory with NMPC');
xlabel('X Position');
ylabel('Y Position');
legend('Trajectory', 'Start', 'Waypoint');
grid on;

figure;
plot(t, rad2deg(state(:, 3)), 'LineWidth', 2);
title('Sailboat Heading Over Time');
xlabel('Time (s)');
ylabel('Heading (deg)');
grid on;

figure;
plot(t, state(:, 4), 'LineWidth', 2);
title('Sailboat Speed Over Time');
xlabel('Time (s)');
ylabel('Speed (m/s)');
grid on;

figure;
plot(t, delta_r_deg, 'LineWidth', 2);
title('Rudder Angle Over Time (NMPC)');
xlabel('Time (s)');
ylabel('Rudder Angle (deg)');
grid on;

figure;
plot(t, delta_s_deg, 'LineWidth', 2);
title('Sail Angle Over Time (NMPC)');
xlabel('Time (s)');
ylabel('Sail Angle (deg)');
grid on;

%% NMPC Cost Function Definition
function J = nmpc_cost(U_vec, x0, waypoint, params, wind, currents, dt, N_horizon)
    % NMPC cost function using direct shooting:
    % U_vec contains the sequence of control inputs over the horizon.
    % x0 is the current state; waypoint is the desired [x, y] position.
    % The cost penalizes the squared distance from the waypoint and control effort.
    
    nu = 2;  % control input dimension: [delta_r; delta_s]
    x = x0;  % initialize predicted state with current state
    J = 0;   % initialize cost
    Q = 1;   % weight for position error (can be tuned)
    R = 0.1; % weight for control effort (can be tuned)
    
    % Roll out the dynamics over the prediction horizon using Euler integration
    for i = 1:N_horizon
        % Extract the control inputs for step i
        delta_r = U_vec((i-1)*nu + 1);
        delta_s = U_vec((i-1)*nu + 2);
        controls = struct('delta_r', delta_r, 'delta_s', delta_s);
        
        % Simulate dynamics; here, time argument is arbitrary for prediction (using dt)
        dx = sailboat_dynamics_old(0, x, params, controls, wind, currents);
        x = x + dt * dx;
        
        % Compute stage cost: position error plus control effort
        pos_error = (x(1) - waypoint(1))^2 + (x(2) - waypoint(2))^2;
        control_effort = delta_r^2 + delta_s^2;
        J = J + Q * pos_error + R * control_effort;
    end
end
