function run_simulation()
    % Initialize parameters
    time_span = 0:0.1:60; % Simulation time from 0 to 60 seconds
    initial_conditions = [0; 0; 0; 0]; % [x_position; y_position; heading; sail_angle]
    
    % Load models
    simplified_model = @simplified_model;
    high_fidelity_model = @high_fidelity_model;

    % Set up environmental conditions
    wind_strength = 10; % Example wind strength
    wind_direction = 45; % Example wind direction in degrees
    current_strength = 2; % Example ocean current strength
    current_direction = 90; % Example current direction in degrees

    % Run simulations
    [t_simplified, state_simplified] = ode45(@(t, state) simplified_model(t, state, wind_strength, wind_direction, current_strength, current_direction), time_span, initial_conditions);
    [t_high_fidelity, state_high_fidelity] = ode45(@(t, state) high_fidelity_model(t, state, wind_strength, wind_direction, current_strength, current_direction), time_span, initial_conditions);

    % Visualize results
    visualize_results(t_simplified, state_simplified, 'Simplified Model');
    visualize_results(t_high_fidelity, state_high_fidelity, 'High Fidelity Model');
end

function visualize_results(t, state, model_name)
    figure;
    plot(state(:, 1), state(:, 2));
    title(['Trajectory of Sailboat - ', model_name]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
end