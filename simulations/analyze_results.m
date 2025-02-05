function analyze_results(simulation_data)
    % This function analyzes the results of the sailboat simulations.
    % Input:
    %   simulation_data - Struct containing simulation results including
    %   position, velocity, heading, and control inputs over time.

    % Extract relevant data
    time = simulation_data.time;
    position = simulation_data.position;
    velocity = simulation_data.velocity;
    heading = simulation_data.heading;

    % Calculate performance metrics
    final_position = position(end, :);
    average_velocity = mean(velocity, 1);
    total_distance = sum(sqrt(diff(position(:, 1)).^2 + diff(position(:, 2)).^2));

    % Display metrics
    fprintf('Final Position: (%.2f, %.2f)\n', final_position(1), final_position(2));
    fprintf('Average Velocity: (%.2f, %.2f)\n', average_velocity(1), average_velocity(2));
    fprintf('Total Distance Traveled: %.2f\n', total_distance);

    % Visualization
    figure;
    plot(position(:, 1), position(:, 2), 'b-', 'LineWidth', 2);
    title('Sailboat Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
    axis equal;
end