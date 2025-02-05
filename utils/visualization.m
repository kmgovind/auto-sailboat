function visualize_trajectory(time, position, heading)
    % Visualize the sailboat's trajectory over time
    figure;
    subplot(2, 1, 1);
    plot(time, position(:, 1), 'r', 'LineWidth', 2); % X position
    hold on;
    plot(time, position(:, 2), 'b', 'LineWidth', 2); % Y position
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Sailboat Trajectory');
    legend('X Position', 'Y Position');
    grid on;

    subplot(2, 1, 2);
    plot(time, heading, 'g', 'LineWidth', 2); % Heading
    xlabel('Time (s)');
    ylabel('Heading (rad)');
    title('Sailboat Heading Over Time');
    grid on;
end

function visualize_results(simulation_data)
    % Visualize simulation results including trajectory and control inputs
    time = simulation_data.time;
    position = simulation_data.position;
    heading = simulation_data.heading;

    visualize_trajectory(time, position, heading);

    % Additional visualizations can be added here
end