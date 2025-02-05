function new_path = replanning_module(current_position, waypoints, obstacles)
    % This function adjusts the sailboat's path in response to detected obstacles.
    
    % Parameters:
    % current_position: Current position of the sailboat [x, y]
    % waypoints: Array of waypoints to follow [x1, y1; x2, y2; ...]
    % obstacles: Array of detected obstacles [x1, y1; x2, y2; ...]
    
    % Initialize new path as the original waypoints
    new_path = waypoints;
    
    % Check for obstacles and adjust the path
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        
        % Calculate distance from current position to obstacle
        distance_to_obstacle = norm(current_position - obstacle);
        
        % If the obstacle is within a certain threshold, replanning is needed
        if distance_to_obstacle < 5 % threshold distance
            % Adjust waypoints to avoid the obstacle
            for j = 1:size(new_path, 1)
                % Simple avoidance strategy: shift waypoints away from the obstacle
                direction = new_path(j, :) - obstacle;
                if norm(direction) > 0
                    direction = direction / norm(direction); % Normalize direction
                    new_path(j, :) = new_path(j, :) + 2 * direction; % Shift away
                end
            end
        end
    end
end