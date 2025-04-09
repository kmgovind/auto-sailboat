function [delta_r, delta_s] = calculate_sail_and_rudder_deflections(current_speed, current_direction, wind_speed, wind_direction, desired_heading, params)
    % Calculate the necessary sail and rudder deflections to achieve the desired heading
    % Inputs:
    %   current_speed: Ocean current speed (m/s)
    %   current_direction: Ocean current direction (radians)
    %   wind_speed: Wind speed (m/s)
    %   wind_direction: Wind direction (radians)
    %   desired_heading: Desired boat heading (radians)
    %   params: Sailboat parameters (e.g., max rudder angle, max sail angle)
    % Outputs:
    %   delta_r: Rudder deflection angle (radians)
    %   delta_s: Sail deflection angle (radians)

    % Compute the apparent wind velocity in the boat's reference frame
    apparent_wind_x = wind_speed * cos(wind_direction) - current_speed * cos(current_direction);
    apparent_wind_y = wind_speed * sin(wind_direction) - current_speed * sin(current_direction);
    apparent_wind_speed = sqrt(apparent_wind_x^2 + apparent_wind_y^2);
    apparent_wind_angle = atan2(apparent_wind_y, apparent_wind_x);

    % Compute the angle of attack for the sail
    angle_of_attack_sail = wrapToPi(apparent_wind_angle - desired_heading);

    % Determine sail deflection angle (delta_s)
    max_sail_angle = params.max_sail_angle; % Maximum allowable sail angle (radians)
    delta_s = max(-max_sail_angle, min(max_sail_angle, angle_of_attack_sail));

    % Compute the desired rudder angle to align the boat's heading with the desired heading
    heading_error = wrapToPi(desired_heading - apparent_wind_angle);
    max_rudder_angle = params.max_rudder_angle; % Maximum allowable rudder angle (radians)
    delta_r = max(-max_rudder_angle, min(max_rudder_angle, heading_error));

    % Ensure the rudder and sail angles are within their physical limits
    delta_r = max(-max_rudder_angle, min(max_rudder_angle, delta_r));
    delta_s = max(-max_sail_angle, min(max_sail_angle, delta_s));
end