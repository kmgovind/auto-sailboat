function new_path = replanning_module(current_position, waypoints, wind_direction)
    % This function follows a racecourse and adjusts the sailboat's path based on the wind direction.
    
    % Parameters:
    % current_position: Current position of the sailboat [x, y]
    % waypoints: Array of waypoints to follow [x1, y1; x2, y2; ...]
    % wind_direction: The direction of the wind in degrees (0 to 360)
    
    % Assumptions:
    % wind_direction: Wind direction is given in degrees from north
    
    % Initialize new path as the original waypoints
    new_path = waypoints;
    
    % Convert wind direction to a unit vector
    wind_rad = deg2rad(wind_direction);
    wind_vec = [cos(wind_rad), sin(wind_rad)];
   
    % Adjust waypoints based on wind direction
    for j = 1:size(new_path, 1)
    to_waypoint = new_path(j, :) - current_position;
    to_waypoint_norm = to_waypoint / norm(to_waypoint); % Normalize direction to waypoint
    
    % Compute dot product manually
    dot_product = sum(to_waypoint_norm .* wind_vec); % Element-wise multiplication and summation
    
    % Check if waypoint is directly upwind
        if dot_product > cosd(45) % Allow some margin (e.g., 45 degrees)
            % Adjust waypoint to avoid directly upwind
            perp_vec = [-wind_vec(2), wind_vec(1)]; % Perpendicular vector to wind direction
            new_path(j, :) = new_path(j, :) + 2 * perp_vec; % Shift away from upwind direction
        end
    end

end
