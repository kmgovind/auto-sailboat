function run_sailboat_simulation()
    % Define simulation parameters
    time_span = [0 100]; % Simulation time span
    initial_conditions = [0; 0; 0; 1; 0]; % Initial state [x; y; theta; v; omega]

    % Define model parameters
    params = struct('p1', 0.1, 'p2', 0.01, 'p3', 0.1, 'p6', 0.5, 'p7', 0.3, ...
                    'p8', 0.2, 'p9', 1.5, 'p10', 0.5, 'p11', 0.1, ...
                    'g_s', 1.0, 'g_r', 0.5);

    % Define control inputs
    controls = struct('delta_s', pi/6, 'delta_r', pi/12);

    % Define wind conditions
    wind = struct('a_tw', 1.0, 'psi_tw', pi/4);

    % Define ocean currents
    currents = struct('v_cx', 0.5, 'v_cy', 0.2);

    % Run the simulation
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
    [X, Y] = meshgrid(0:1:10, 0:1:10);
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
end
