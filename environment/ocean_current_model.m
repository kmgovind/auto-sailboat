function current = ocean_current_model(position, time)
    % Ocean current model simulating the effects of ocean currents on the sailboat's trajectory.
    
    % Define current strength and direction (example values)
    current_strength = 1.0; % in m/s
    current_direction = pi / 4; % 45 degrees in radians
    
    % Calculate current vector based on position and time
    % Here we can introduce variability based on time or position if needed
    current_x = current_strength * cos(current_direction);
    current_y = current_strength * sin(current_direction);
    
    % Return the current as a vector
    current = [current_x; current_y];
end