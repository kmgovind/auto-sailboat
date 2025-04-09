function run_sailboat_simulation_with_controller()
    % Ready workspace
    clc; close all;

    % Simulation parameters
    time_span = [0 3000]; % run 300s for enough time
    % dt = 0.1;
    initial_conditions = [100; 0; pi/2; 0; 0];  % [x; y; theta; v; omega]

    % Model params
    params = define_params();

    % Ocean currents
    currents = struct('v_cx', 0.0, 'v_cy', 0.0);

    % Set wind to 7 m/s from West (π)
    wind = struct( ...
        't',       [time_span(1), time_span(end)], ...  % minimal time array
        'a_tw',    [7, 7],             ...  % always 7 m/s
        'psi_tw',  [pi, pi]            ...  % always pi (west -> east)
    );

    % Waypoints
    waypoints = [
         0  -100
         0     0
        -100 -400
         -50 -600
        -100 -200
        -150  100
    ];

    [t, state] = ode45(@(t, st) sailboat_dynamics(t, st, params, ...
                                melin_controller_wrapper(st, waypoints), ...
                                wind, currents), ...
                       time_span, initial_conditions);

    % --- Plot Final Results ---
    % **Sailboat Trajectory Plot**
    figure;
    hold on;
    plot(state(:,1), state(:,2), 'b', 'LineWidth', 2);
    plot(waypoints(:,1), waypoints(:,2), 'ks', 'MarkerFaceColor','y', 'MarkerSize', 8); % Waypoints
    plot(100, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start Position
    title('Sailboat Trajectory (Melin Sin-Controller)');
    xlabel('X Position'); ylabel('Y Position');
    grid on;
    legend('Trajectory', 'Waypoints', 'Start');

    % **Heading Over Time Plot**
    figure;
    plot(t, rad2deg(state(:,3)), 'r', 'LineWidth', 2);
    title('Sailboat Heading Over Time');
    xlabel('Time (s)'); ylabel('Heading (degrees)');
    grid on;
    legend('Boat Heading');

    % **Ocean Currents Plot**
    figure;
    x_min = min(state(:,1)); x_max = max(state(:,1));
    y_min = min(state(:,2)); y_max = max(state(:,2));
    [X, Y] = meshgrid(linspace(x_min, x_max, 20), linspace(y_min, y_max, 20));
    U = currents.v_cx * ones(size(X));
    V = currents.v_cy * ones(size(Y));

    quiver(X, Y, U, V); hold on;
    plot(state(:,1), state(:,2), 'r', 'LineWidth', 2);
    title('Ocean Currents and Sailboat Trajectory');
    xlabel('X Position'); ylabel('Y Position');
    grid on;
    legend('Current Field', 'Sailboat Trajectory');

    % **Extract Rudder & Sail Commands**
    rudder_cmd = zeros(length(t),1);
    sail_cmd   = zeros(length(t),1);
    for i = 1:length(t)
       c = melin_controller_wrapper(state(i,:)', waypoints);
       rudder_cmd(i) = c.delta_r;
       sail_cmd(i)   = c.delta_s;
    end

    % **Rudder and Sail Commands Over Time**
    figure;
    subplot(2,1,1);
    plot(t, rudder_cmd, 'LineWidth', 1.5);
    title('Rudder Commands Over Time');
    xlabel('Time (s)'); ylabel('Rudder Angle (rad)');
    grid on;
    legend('Rudder Angle');

    subplot(2,1,2);
    plot(t, sail_cmd, 'LineWidth', 1.5);
    title('Sail Commands Over Time');
    xlabel('Time (s)'); ylabel('Sail Angle (rad)');
    grid on;
    legend('Sail Angle');

end

function controls = melin_controller_wrapper(state, waypoints)
    
    persistent wp_index
    if isempty(wp_index)
        wp_index = 1;
    end

    % state is [x; y; theta; v; omega]
    current_position = state(1:2)';
    current_heading  = state(3);
    boat_speed       = state(4);

    % Wind Conditions (as per main script)
    wind_speed     = 7.0;
    wind_direction = pi;

    out = melin_controller(current_position, current_heading, ...
                           boat_speed, wind_speed, wind_direction, ...
                           waypoints, wp_index);

    controls.delta_r = out.delta_r;
    controls.delta_s = out.delta_s;
    wp_index         = out.waypoint_index;  % update for next call
end