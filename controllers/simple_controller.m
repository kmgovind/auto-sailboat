function [delta_r, delta_s] = simple_controller(x, y, theta_b, v_b, x_w, y_w, V_w, psi_w, V_c, psi_c)
    % Constants
    delta_s_min = 0; % Minimum sail angle
    delta_s_max = pi/3; % Maximum sail angle
    delta_r_min = -pi/6; % Minimum rudder angle
    delta_r_max = pi/6; % Maximum rudder angle
    Kp = 1.0; % Proportional gain for rudder control

    % Compute the desired heading to the waypoint
    theta_des = atan2(y_w - y, x_w - x);

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

    % Compute the sail angle relative to the apparent wind
    delta_s = 0.5 * (psi_aw - theta_b);
    delta_s = max(delta_s_min, min(delta_s_max, delta_s));

    % Compute the heading error
    e_theta = wrapToPi(theta_des - theta_b);

    % Compute the rudder angle using a proportional controller
    delta_r = Kp * e_theta;
    delta_r = max(delta_r_min, min(delta_r_max, delta_r));

    % Return control inputs
    delta_r = wrapToPi(delta_r);
    delta_s = wrapToPi(delta_s);
end