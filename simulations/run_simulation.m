function run_simulation(model_name, wind_model_name, current_model_name)
    % run_simulation.m
    % This script runs a simulation for the specified sailboat model in the given environment.

    % Define simulation parameters
    time_span = [0 100]; % Simulation time span
    initial_conditions = [0; 0; 1; 0; 0]; % Initial state [x; y; velocity; heading; sail_angle]

    % Define environmental conditions
    wind_strength = 5; % Example wind strength
    wind_direction = pi/4; % Example wind direction (45 degrees)
    current_strength = 2; % Example current strength
    current_direction = pi/2; % Example current direction (90 degrees)

    % Run the specified model
    if strcmp(model_name, 'simplified_model')
        [t, state] = ode45(@(t, state) simplified_model(t, state, wind_strength, wind_direction, current_strength, current_direction), time_span, initial_conditions);
        model_title = 'Simplified Model Trajectory';
    elseif strcmp(model_name, 'high_fidelity_model')
        [t, state] = ode45(@(t, state) high_fidelity_model(t, state, wind_strength, wind_direction, current_strength, current_direction), time_span, initial_conditions);
        model_title = 'High Fidelity Model Trajectory';
    else
        error('Unknown model name.');
    end

    % Plot results
    figure;
    plot(state(:, 1), state(:, 2));
    title(model_title);
    xlabel('X Position');
    ylabel('Y Position');
end

function visualize_results(t, state, model_name)
    figure;
    plot(state(:, 1), state(:, 2));
    title(['Trajectory of Sailboat - ', model_name]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
end