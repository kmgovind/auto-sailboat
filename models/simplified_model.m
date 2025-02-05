function dstate = simplified_model(t, state, wind_strength, wind_direction, current_strength, current_direction)
    % simplified_model.m
    % This function defines the dynamics of a simplified sailboat model.
    % Inputs:
    %   t - time
    %   state - state vector [x; y; velocity; heading; sail_angle]
    %   wind_strength - strength of the wind
    %   wind_direction - direction of the wind
    %   current_strength - strength of the ocean current
    %   current_direction - direction of the ocean current

    % Extract state variables
    x = state(1);
    y = state(2);
    velocity = state(3);
    heading = state(4);
    sail_angle = state(5);

    % Define sailboat dynamics (simplified example)
    % Compute forces and torques
    wind_force = wind_strength * cos(wind_direction - heading);
    current_force = current_strength * cos(current_direction - heading);

    % Update state derivatives
    dx = velocity * cos(heading);
    dy = velocity * sin(heading);
    dvelocity = wind_force + current_force; % Simplified force model
    dheading = 0; % No change in heading for simplicity
    dsail_angle = 0; % No change in sail angle for simplicity

    % Return state derivatives
    dstate = [dx; dy; dvelocity; dheading; dsail_angle];
end