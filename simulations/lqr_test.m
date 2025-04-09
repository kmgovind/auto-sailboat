clear;clc;close all;

% Define simulation parameters
time_span = [0 100]; % Simulation time span
dt = 0.1; % Time step size
t = time_span(1):dt:time_span(2); % Time vector
N = length(t); % Number of time steps

% Initial state [x; y; theta; v; omega]
initial_conditions = [0; 0; 0; 1.5; 0];

% Get model parameters
params = define_params();

% Define ocean currents
currents = struct('v_cx', 1.0, 'v_cy', 0.2);

% Define initial wind conditions
initial_wind_speed = 5.0;  % m/s
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

% Linearize the sailboat dynamics around the initial conditions
[A, B] = linearize_model(initial_conditions, struct('delta_s', 1, 'delta_r', 0), params, wind, currents);

% Check whether system is controllable
Co = ctrb(A, B)

unco = length(A) - rank(Co)
if unco > 0
    error("System is uncontrollable in %d states", unco)
end
