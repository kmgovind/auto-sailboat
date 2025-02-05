function [state_dot] = high_fidelity_model(state, control_input, wind, ocean_current)
    % State variables
    % state = [x; y; heading; velocity; sail_angle]
    x = state(1);
    y = state(2);
    heading = state(3);
    velocity = state(4);
    sail_angle = state(5);
    
    % Constants
    mass = 1.5; % mass of the sailboat (kg)
    drag_coefficient = 0.1; % drag coefficient
    lift_coefficient = 1.2; % lift coefficient for sails
    water_density = 1025; % density of seawater (kg/m^3)
    sail_area = 0.5; % sail area (m^2)

    % Wind dynamics
    wind_force = wind_model(wind, sail_angle);
    
    % Ocean current dynamics
    current_force = ocean_current_model(ocean_current);
    
    % Calculate total forces
    total_force_x = wind_force(1) + current_force(1) - drag_coefficient * velocity^2 * cos(heading);
    total_force_y = wind_force(2) + current_force(2) - drag_coefficient * velocity^2 * sin(heading);
    
    % Calculate acceleration
    ax = total_force_x / mass;
    ay = total_force_y / mass;

    % Update state variables
    state_dot = zeros(5, 1);
    state_dot(1) = velocity * cos(heading); % dx/dt
    state_dot(2) = velocity * sin(heading); % dy/dt
    state_dot(3) = (total_force_y * cos(heading) - total_force_x * sin(heading)) / (mass * velocity); % dheading/dt
    state_dot(4) = sqrt(ax^2 + ay^2); % dvelocity/dt
    state_dot(5) = control_input; % dsail_angle/dt
end

function wind_force = wind_model(wind, sail_angle)
    % Wind model calculations
    wind_speed = wind(1);
    wind_direction = wind(2);
    
    % Calculate lift and drag forces based on wind and sail angle
    lift = 0.5 * water_density * wind_speed^2 * sail_area * lift_coefficient * sin(sail_angle);
    drag = 0.5 * water_density * wind_speed^2 * sail_area * drag_coefficient * cos(sail_angle);
    
    % Convert to x and y components
    wind_force = [lift * cos(wind_direction) - drag * sin(wind_direction);
                  lift * sin(wind_direction) + drag * cos(wind_direction)];
end

function current_force = ocean_current_model(current)
    % Ocean current model calculations
    current_speed = current(1);
    current_direction = current(2);
    
    % Calculate current force
    current_force = [current_speed * cos(current_direction);
                     current_speed * sin(current_direction)];
end