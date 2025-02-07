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
    drag_coefficient = 0.1; % hull drag coefficient
    lift_coefficient = 1.2; % lift coefficient for sails
    water_density = 1025; % density of seawater (kg/m^3)
    sail_area = 0.5; % sail area (m^2)
    air_density = 1.225; % density of air (kg/m^3)

    % Compute wind force using NACA 0015 aerodynamic model
    wind_force = wind_model(wind, state, sail_angle, sail_area, air_density);
    
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

function wind_force = wind_model(wind, state, sail_angle, sail_area, air_density)
    % Wind model calculations
    wind_speed = wind(1);
    wind_direction = wind(2);
    
    % Compute Apparent Wind (Wind Relative to Boat Motion)
    boat_speed = state(4);
    boat_heading = state(3);
    
    % Boat velocity components
    vx_boat = boat_speed * cos(boat_heading);
    vy_boat = boat_speed * sin(boat_heading);

    % Wind velocity components
    vx_wind = wind_speed * cos(wind_direction);
    vy_wind = wind_speed * sin(wind_direction);

    % Apparent wind velocity (wind relative to the boat)
    vx_app = vx_wind - vx_boat;
    vy_app = vy_wind - vy_boat;
    wind_speed_app = sqrt(vx_app^2 + vy_app^2);
    wind_direction_app = atan2(vy_app, vx_app);

    % --- NACA 0015 calculations begins here ---
    persistent alpha_data Cl_data Cd_data
    if isempty(alpha_data)
        data = readtable('xf-naca0015-il-50000.csv');
        % Data from: http://airfoiltools.com/polar/details?polar=xf-naca0015-il-50000
        alpha_data = data.Alpha; 
        Cl_data    = data.Cl;
        Cd_data    = data.Cd;
    end
    
    % Compute Angle of Attack from apparent wind
    alpha_radians = wind_direction_app - (boat_heading + sail_angle);
    alpha_degs = alpha_radians * (180/pi);
    
    % Interpolate real Cl & Cd
    Cl = interp1(alpha_data, Cl_data, alpha_degs, 'linear', 'extrap');
    Cd = interp1(alpha_data, Cd_data, alpha_degs, 'linear', 'extrap');
    
    % Compute lift & drag
    L = 0.5 * air_density * wind_speed_app^2 * sail_area * Cl;
    D = 0.5 * air_density * wind_speed_app^2 * sail_area * Cd;
    % --- NACA 0015 calculations ends here ---
    
    % Convert lift and drag to X and Y components
    lift_angle = wind_direction_app + pi/2;  % Lift : perpendicular to wind
    drag_angle = wind_direction_app;         % Drag : parallel to wind

    Fx_lift = L * cos(lift_angle);
    Fy_lift = L * sin(lift_angle);

    Fx_drag = D * cos(drag_angle);
    Fy_drag = D * sin(drag_angle);

    % Net wind force on sail
    wind_force = [Fx_lift - Fx_drag;
                  Fy_lift - Fy_drag];
end

function current_force = ocean_current_model(current)
    % Ocean current model calculations
    current_speed = current(1);
    current_direction = current(2);
    
    % Calculate current force
    current_force = [current_speed * cos(current_direction);
                     current_speed * sin(current_direction)];
end