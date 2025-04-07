function [delta_r, delta_s] = lower_level_control( ...
    x, y, theta_b, v_b, ...
    x_w, y_w, ...
    V_w, psi_w, ...
    v_cx, v_cy)
%LOWER_LEVEL_CONTROL  Computes rudder (delta_r) and sail (delta_s) angles 
% based on waypoint navigation and the wind/current conditions.
%
% Inputs:
%   x, y      - Current position of the boat
%   theta_b   - Current heading (radians)
%   v_b       - Boat's forward speed in its heading direction (m/s)
%   x_w, y_w  - Waypoint coordinates
%   V_w       - True wind speed (m/s)
%   psi_w     - True wind direction (radians)
%   v_cx, v_cy - Current velocity components (m/s)
%
% Outputs:
%   delta_r   - Rudder angle (radians)
%   delta_s   - Sail angle (radians)

    % --- Tunable Controller/Boat Parameters ---
    delta_s_min = -pi/6;   % Minimum sail angle
    delta_s_max =  pi/6;   % Maximum sail angle
    delta_r_min = -pi/6;   % Minimum rudder angle
    delta_r_max =  pi/6;   % Maximum rudder angle
    Kp = 1.0;              % Proportional gain for rudder control

    % --- 1) Desired Heading to the Waypoint ---
    theta_des = atan2(y_w - y, x_w - x); 
    theta_des = mod(theta_des, 2*pi);    % ensure angle is in [0, 2*pi]

    % --- 2) Effective Boat Velocity (boat + current) ---
    % The boat moves forward at v_b along heading theta_b, plus the current
    V_eff_x = v_b * cos(theta_b) + v_cx;
    V_eff_y = v_b * sin(theta_b) + v_cy;

    % --- 3) Apparent Wind (vector difference) ---
    % The boat experiences an apparent wind that is the true wind minus V_eff.
    V_aw_x = V_w * cos(psi_w) - V_eff_x;
    V_aw_y = V_w * sin(psi_w) - V_eff_y;

    % Apparent wind angle
    psi_aw = atan2(V_aw_y, V_aw_x);
    psi_aw = mod(psi_aw, 2*pi);

    % --- 4) Sail Angle ---
    % Simple approach: position sail half-way between heading and apparent wind.
    delta_s = 0.5 * (psi_aw - theta_b);
    % Clamp to allowed sail angles
    delta_s = max(delta_s_min, min(delta_s_max, delta_s));

    % --- 5) Rudder Angle (heading control) ---
    % Heading error e_theta in [-pi, pi]
    e_theta = mod(theta_des - theta_b + pi, 2*pi) - pi;

    % Proportional rudder control
    delta_r = Kp * e_theta;
    % Clamp to allowed rudder angles
    delta_r = max(delta_r_min, min(delta_r_max, delta_r));

    % --- 6) Normalize angles to [-pi, pi] for output ---
    delta_r = mod(delta_r + pi, 2*pi) - pi;
    delta_s = mod(delta_s + pi, 2*pi) - pi;
end