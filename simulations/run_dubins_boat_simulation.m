function run_dubins_boat_simulation()
    % Define simulation parameters
    time_span = [0 100]; % Simulation time span
    initial_conditions = [0; 0; 0; 1]; % Initial state [x; y; theta; v]simulations/run_dubins_boat_simulation.m

    % Define control inputs
    controls = struct('delta_r', pi/12); % Rudder angle

    % Define wind conditions
    wind = struct('a_tw', 1.0, 'psi_tw', pi/4);

    % Run the simulation
    [t, state] = ode45(@(t, state) dubins_boat_model(t, state, controls, wind), time_span, initial_conditions);

    % Plot results
    figure;
    plot(state(:, 1), state(:, 2));
    title('Dubins Boat Trajectory');
    xlabel('X Position');
    ylabel('Y Position');
    grid on;

    figure;
    plot(t, state(:, 3));
    title('Dubins Boat Heading Over Time');
    xlabel('Time (s)');
    ylabel('Heading (rad)');
    grid on;
end