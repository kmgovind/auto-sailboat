function wind_currents = wind_current_model(time, wind_strength, wind_direction)
    % wind_model simulates high-fidelity wind dynamics affecting the sailboat.
    % Inputs:
    %   time - current time in the simulation
    %   wind_strength - initial wind speed (m/s)
    %   wind_direction - initial wind direction (degrees)
    
    % Convert wind direction from degrees to radians
    wind_direction_rad = deg2rad(wind_direction);
    
    % Define gust parameters
    gust_amplitude = 5;  % Max gust speed change (m/s)
    gust_frequency = 0.2;  % How often gusts occur (Hz)
    
    % Define turbulence parameters (small random variations)
    turbulence_amplitude = 0.3;  % Max variation (m/s)
    
    % Define slow-changing wind drift (long-term trend)
    drift_amplitude = 0.5;  % Max long-term change (m/s)
    drift_frequency = 0.01;  % Very slow drift (Hz)

    % Simulate dynamic wind strength
    gust_effect = gust_amplitude * sin(gust_frequency * time);
    turbulence_effect = turbulence_amplitude * (randn(1) - 0.5);
    drift_effect = drift_amplitude * sin(drift_frequency * time);
    
    wind_speed = wind_strength + gust_effect + turbulence_effect + drift_effect;
    
    % Simulate dynamic wind direction
    direction_drift = deg2rad(20) * sin(0.1 * time); % usage: 20 degrees max drift, 0.1 rad/s
    turbulence_direction = deg2rad(10) * (randn(1) - 0.5); % usage: 10 degrees max variation, random change in direction

    wind_direction_rad = wind_direction_rad + direction_drift + turbulence_direction;
    
    % Compute wind components
    wind_currents.x = wind_speed * cos(wind_direction_rad); % East-West component
    wind_currents.y = wind_speed * sin(wind_direction_rad); % North-South component

    % Compute current components
    % Currents are 20-45 deg to the right of the wind direction, and 
    
    % Store wind properties
    wind_currents.speed = wind_speed;
    wind_currents.direction = rad2deg(wind_direction_rad); % Convert back to degrees
    wind_currents.timestamp = time;
end