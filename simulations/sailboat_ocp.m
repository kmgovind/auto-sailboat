function sailboat_ocp()
    % Define initial and final states
    x0 = 0; y0 = 0; theta0 = 0; v0 = 0; omega0 = 0;
    xf = 10; yf = 10;
    initial_state = [x0, y0, theta0, v0, omega0];
    final_position = [xf, yf];

    % Define control limits
    delta_r_min = -pi/6; delta_r_max = pi/6;
    delta_s_min = 0; delta_s_max = pi/3;

    % Define no-go zone constraint
    theta_no_go = pi/6;

    % Define time discretization
    N = 50; % Number of time steps
    dt = 0.1; % Time step size

    % Initial guess for decision variables
    x_guess = linspace(x0, xf, N);
    y_guess = linspace(y0, yf, N);
    theta_guess = linspace(theta0, theta0, N);
    v_guess = linspace(v0, v0, N);
    omega_guess = linspace(omega0, omega0, N);
    delta_s_guess = linspace(delta_s_min, delta_s_max, N);
    delta_r_guess = linspace(delta_r_min, delta_r_max, N);
    T_guess = N * dt;

    % Combine all decision variables into a single vector
    decision_vars_guess = [x_guess, y_guess, theta_guess, v_guess, omega_guess, delta_s_guess, delta_r_guess, T_guess];

    % Define bounds for decision variables
    lb = [-inf * ones(1, 5 * N), delta_s_min * ones(1, N), delta_r_min * ones(1, N), 0];
    ub = [inf * ones(1, 5 * N), delta_s_max * ones(1, N), delta_r_max * ones(1, N), inf];

    % Get model parameters
    params = define_params();

    % Define ocean currents
    currents = struct('v_cx', 0.5, 'v_cy', 1.3);

    % Define initial wind conditions
    initial_wind_speed = 5.0;  % m/s
    initial_wind_direction = 45;  % degrees

    % Define time vector for wind simulation
    wind_time = linspace(0, N*dt, N); % Higher resolution for gusts

    % Simulate wind conditions over time
    wind_speeds = zeros(size(wind_time));
    wind_directions = zeros(size(wind_time));

    for i = 1:length(wind_time)
        wind_instance = enhanced_wind_model(wind_time(i), initial_wind_speed, initial_wind_direction);
        wind_speeds(i) = wind_instance.speed;
        wind_directions(i) = wind_instance.direction;
    end

    % Ensure wind data has at least two points for interpolation
    if length(wind_time) < 2
        wind_time = [0, dt];
        wind_speeds = [initial_wind_speed, initial_wind_speed];
        wind_directions = [initial_wind_direction, initial_wind_direction];
    end

    % Store wind structure for interpolation
    wind = struct('t', wind_time, 'a_tw', wind_speeds, 'psi_tw', deg2rad(wind_directions));

    % Set up optimization problem
    options = optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 1e5);
    [opt_decision_vars, ~] = fmincon(@objective_function, decision_vars_guess, [], [], [], [], lb, ub, @(vars) constraints(vars, initial_state, final_position, params, wind, currents, N, dt, theta_no_go), options);

    % Extract optimized states and controls
    x_opt = opt_decision_vars(1:N);
    y_opt = opt_decision_vars(N+1:2*N);
    theta_opt = opt_decision_vars(2*N+1:3*N);
    v_opt = opt_decision_vars(3*N+1:4*N);
    omega_opt = opt_decision_vars(4*N+1:5*N);
    delta_s_opt = opt_decision_vars(5*N+1:6*N);
    delta_r_opt = opt_decision_vars(6*N+1:7*N);
    T_opt = opt_decision_vars(end);

    % Plot optimized trajectory
    figure;
    plot(x_opt, y_opt, 'b-', 'LineWidth', 2);
    hold on;
    plot(x0, y0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(xf, yf, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    title('Optimized Sailboat Trajectory');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Trajectory', 'Start', 'Goal');
    grid on;
end

function J = objective_function(vars)
    % Objective function to minimize final time
    J = vars(end);
end

function [c, ceq] = constraints(vars, initial_state, final_position, params, wind, currents, N, dt, theta_no_go)
    % Extract states and controls from decision variables
    x = vars(1:N);
    y = vars(N+1:2*N);
    theta = vars(2*N+1:3*N);
    v = vars(3*N+1:4*N);
    omega = vars(4*N+1:5*N);
    delta_s = vars(5*N+1:6*N);
    delta_r = vars(6*N+1:7*N);
    T = vars(end);

    % Initialize constraints
    c = [];
    ceq = [];

    % Dynamics constraints using Euler discretization
    for k = 1:N-1
        state_k = [x(k); y(k); theta(k); v(k); omega(k)];
        state_k1 = [x(k+1); y(k+1); theta(k+1); v(k+1); omega(k+1)];
        control_k = struct('delta_s', delta_s(k), 'delta_r', delta_r(k));
        wind_k = struct('t', wind.t, 'a_tw', wind.a_tw(k), 'psi_tw', wind.psi_tw(k));
        dstate_k = sailboat_dynamics_ocp(0, state_k, params, control_k, wind_k, currents);
        ceq = [ceq; state_k1 - (state_k + dt * dstate_k)];
    end

    % Initial condition constraint
    ceq = [ceq; x(1) - initial_state(1); y(1) - initial_state(2); theta(1) - initial_state(3); v(1) - initial_state(4); omega(1) - initial_state(5)];

    % Final position constraint
    ceq = [ceq; x(end) - final_position(1); y(end) - final_position(2)];

    % No-go zone constraint
    for k = 1:N
        c = [c; abs(theta(k) - wind.psi_tw(k)) - theta_no_go];
    end
end