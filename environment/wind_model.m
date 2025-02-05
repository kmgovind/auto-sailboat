function wind = wind_model(time, wind_strength, wind_direction)
    % wind_model simulates the wind dynamics affecting the sailboat.
    % Inputs:
    %   time - current time in the simulation
    %   wind_strength - strength of the wind (m/s)
    %   wind_direction - direction of the wind (degrees)

    % Convert wind direction from degrees to radians
    wind_direction_rad = deg2rad(wind_direction);
    
    % Calculate wind components
    wind.x = wind_strength * cos(wind_direction_rad); % East-West component
    wind.y = wind_strength * sin(wind_direction_rad); % North-South component
    
    % Optional: Add time-varying wind dynamics (e.g., gusts)
    % Example: wind_strength = wind_strength + 0.1 * sin(0.1 * time);
    
    % Return wind structure
    wind.timestamp = time;
end