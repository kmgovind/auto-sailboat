function controls = simple_controller(desired_heading, current_heading)
    % Simple proportional controller for rudder angle
    Kp = 1.0; % Proportional gain
    heading_error = desired_heading - current_heading;
    delta_r = Kp * heading_error;

    % Limit the rudder angle to a maximum value
    max_rudder_angle = pi/6; % 30 degrees
    delta_r = max(min(delta_r, max_rudder_angle), -max_rudder_angle);

    % Return control inputs
    controls = struct('delta_s', pi/6, 'delta_r', delta_r);
end