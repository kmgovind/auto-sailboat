function [state_dot] = simplified_model(state, control_input, wind, current)
    % State variables
    % state = [x; y; velocity; heading; sail_angle]
    x = state(1);
    y = state(2);
    velocity = state(3);
    heading = state(4);
    sail_angle = state(5);
    
    % Constants
    mass = 1.0; % mass of the sailboat
    drag_coefficient = 0.1; % drag coefficient
    wind_force = wind_model(wind, sail_angle); % wind force calculation
    current_force = ocean_current_model(current); % current force calculation
    
    % Forces
    total_force_x = wind_force(1) + current_force(1) - drag_coefficient * velocity * cos(heading);
    total_force_y = wind_force(2) + current_force(2) - drag_coefficient * velocity * sin(heading);
    
    % Dynamics
    acceleration_x = total_force_x / mass;
    acceleration_y = total_force_y / mass;
    
    % Update state
    state_dot = zeros(5, 1);
    state_dot(1) = velocity * cos(heading); % dx/dt
    state_dot(2) = velocity * sin(heading); % dy/dt
    state_dot(3) = sqrt(acceleration_x^2 + acceleration_y^2); % dvelocity/dt
    state_dot(4) = atan2(acceleration_y, acceleration_x); % dheading/dt
    state_dot(5) = control_input; % dsail_angle/dt
end

function wind_force = wind_model(wind, sail_angle)
    % Calculate wind force based on wind speed and direction
    wind_speed = wind(1);
    wind_direction = wind(2);
    
    % Simplified wind force calculation
    wind_force_x = wind_speed * cos(wind_direction + sail_angle);
    wind_force_y = wind_speed * sin(wind_direction + sail_angle);
    
    wind_force = [wind_force_x; wind_force_y];
end

function current_force = ocean_current_model(current)
    % Calculate ocean current force
    current_speed = current(1);
    current_direction = current(2);
    
    % Simplified current force calculation
    current_force_x = current_speed * cos(current_direction);
    current_force_y = current_speed * sin(current_direction);
    
    current_force = [current_force_x; current_force_y];
end