function [sail_angle, rudder_angle] = computeSailAndRudderAngles(ocean_speed, ocean_heading, wind_speed, wind_heading, desired_heading)
    % Compute the sail and rudder angles to achieve the desired resultant heading.
    %
    % Parameters:
    %   ocean_speed (double): Speed of the ocean current (in knots or m/s).
    %   ocean_heading (double): Heading of the ocean current (in degrees, 0-360).
    %   wind_speed (double): Speed of the wind (in knots or m/s).
    %   wind_heading (double): Heading of the wind (in degrees, 0-360).
    %   desired_heading (double): Desired resultant heading (in degrees, 0-360).
    %
    % Returns:
    %   sail_angle (double): Sail angle (in degrees).
    %   rudder_angle (double): Rudder angle (in degrees).

    % Convert headings to radians
    ocean_heading_rad = deg2rad(ocean_heading);
    wind_heading_rad = deg2rad(wind_heading);
    desired_heading_rad = deg2rad(desired_heading);

    % Compute ocean current vector
    ocean_x = ocean_speed * cos(ocean_heading_rad);
    ocean_y = ocean_speed * sin(ocean_heading_rad);

    % Compute wind vector
    wind_x = wind_speed * cos(wind_heading_rad);
    wind_y = wind_speed * sin(wind_heading_rad);

    % Desired resultant vector direction
    desired_x = cos(desired_heading_rad);
    desired_y = sin(desired_heading_rad);

    % Compute the boat's required vector to achieve the desired resultant heading
    boat_x = desired_x - ocean_x - wind_x;
    boat_y = desired_y - ocean_y - wind_y;

    % Compute the boat's heading
    boat_heading = atan2(boat_y, boat_x);
    boat_heading_deg = rad2deg(boat_heading);
    boat_heading_deg = mod(boat_heading_deg, 360); % Normalize to [0, 360)

    % Compute sail angle (relative to the wind direction)
    sail_angle = mod(wind_heading - boat_heading_deg, 360);
    if sail_angle > 180
        sail_angle = sail_angle - 360; % Normalize to [-180, 180]
    end

    % Compute rudder angle (relative to the boat's heading)
    rudder_angle = mod(desired_heading - boat_heading_deg, 360);
    if rudder_angle > 180
        rudder_angle = rudder_angle - 360; % Normalize to [-180, 180]
    end
end