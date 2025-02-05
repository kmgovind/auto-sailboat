function control_inputs = waypoint_controller(current_position, current_heading, waypoints, waypoint_index)
    % Constants
    Kp = 1.0; % Proportional gain for heading control
    Kd = 0.5; % Derivative gain for heading control

    % Desired waypoint
    desired_waypoint = waypoints(waypoint_index, :);
    
    % Calculate the desired heading towards the waypoint
    desired_heading = atan2(desired_waypoint(2) - current_position(2), ...
                             desired_waypoint(1) - current_position(1));
    
    % Calculate heading error
    heading_error = desired_heading - current_heading;
    
    % Normalize heading error to the range [-pi, pi]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    % Calculate control inputs
    steering_input = Kp * heading_error; % Proportional control for steering
    control_inputs = struct('steering', steering_input);
    
    % Check if the waypoint is reached
    distance_to_waypoint = norm(desired_waypoint - current_position);
    if distance_to_waypoint < 1.0 % Threshold for reaching the waypoint
        waypoint_index = min(waypoint_index + 1, size(waypoints, 1)); % Move to next waypoint
    end
end