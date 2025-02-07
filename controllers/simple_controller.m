function controls = simple_controller(desired_heading, current_heading, wind_direction, boat_speed, wind_speed, dt)
    % PID gains for rudder control
    persistent prev_error integral_error;
    if isempty(prev_error)
        prev_error = 0;
        integral_error = 0;
    end
    
    Kp = 1.0;   % Proportional gain
    Ki = 0.1;   % Integral gain
    Kd = 0.5;   % Derivative gain
    
    % Compute heading error
    heading_error = wrapToPi(desired_heading - current_heading);
    integral_error = integral_error + heading_error * dt;
    derivative_error = (heading_error - prev_error) / dt;
    prev_error = heading_error;
    
    % Compute rudder angle using PID
    delta_r = Kp * heading_error + Ki * integral_error + Kd * derivative_error;
    
    % Limit rudder angle
    max_rudder_angle = pi/6;
    delta_r = max(min(delta_r, max_rudder_angle), -max_rudder_angle);
    
    % Compute apparent wind angle
    AWA = atan2(sin(wind_direction - current_heading), cos(wind_direction - current_heading) - boat_speed/wind_speed);
    
    % Compute sail angle using optimal trim function
    delta_s = (pi/2) * cos(AWA);
    
    % Limit sail angle
    max_sail_angle = pi/3;
    delta_s = max(min(delta_s, max_sail_angle), 0);
    
    % Return control inputs
    controls = struct('delta_s', delta_s, 'delta_r', delta_r);
end