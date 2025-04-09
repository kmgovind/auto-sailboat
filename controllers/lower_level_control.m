function [delta_r, delta_s] = lower_level_control(x, y, theta_b, v_b, x_w, y_w, V_w, psi_w, V_c, psi_c)
    % Constants
    delta_s_min = -pi/3; % Minimum sail angle
    delta_s_max = pi/3; % Maximum sail angle
    delta_r_min = -pi/6; % Minimum rudder angle
    delta_r_max = pi/6; % Maximum rudder angle
    Kp = 1.0; % Proportional gain for rudder control

    % Compute the desired heading to the waypoint
    theta_des = atan2(y_w - y, x_w - x);
    theta_des = mod(theta_des, 2*pi); % Ensure angle is within [0, 2*pi]

    % Compute the effective velocity of the boat
    V_eff_x = v_b * cos(theta_b) + V_c * cos(psi_c);
    V_eff_y = v_b * sin(theta_b) + V_c * sin(psi_c);
    V_eff = [V_eff_x; V_eff_y];

    % Compute the apparent wind from the true wind velocity
    V_aw_x = V_w * cos(psi_w) - V_eff_x;
    V_aw_y = V_w * sin(psi_w) - V_eff_y;
    V_aw = [V_aw_x; V_aw_y];

    % Compute the apparent wind angle
    psi_aw = atan2(V_aw_y, V_aw_x);
    psi_aw = mod(psi_aw, 2*pi); % Ensure angle is within [0, 2*pi]

    % Compute the sail angle relative to the apparent wind
    delta_s = 0.5 * (psi_aw - theta_b);
    delta_s = max(delta_s_min, min(delta_s_max, delta_s));

    % Compute the heading error
    e_theta = mod(theta_des - theta_b + pi, 2*pi) - pi; % Normalize to [-pi, pi]

    % Compute the rudder angle using a proportional controller
    delta_r = Kp * e_theta;
    delta_r = max(delta_r_min, min(delta_r_max, delta_r));

    % Return control inputs
    delta_r = mod(delta_r + pi, 2*pi) - pi; % Ensure angle is within [-pi, pi]
    delta_s = mod(delta_s + pi, 2*pi) - pi; % Ensure angle is within [-pi, pi]
end