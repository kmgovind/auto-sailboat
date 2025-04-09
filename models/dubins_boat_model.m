function dstate = dubins_boat_model(t, state, controls, wind, currents)
    % Dubins boat model dynamics with heading constraint
    % state = [x; y; theta; v]
    % controls = [delta_r]
    % wind = struct('a_tw', wind_speed, 'psi_tw', wind_direction)

    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3); % Boat heading
    v = state(4);     % Boat velocity

    % Interpolate current components at the current time
    v_cx_interp = interp1(currents.t, currents.v_cx, t, 'linear', 'extrap');
    v_cy_interp = interp1(currents.t, currents.v_cy, t, 'linear', 'extrap');

    % Extract control inputs
    delta_r = controls.delta_r; % Rudder angle

    % Extract wind conditions
    a_tw = wind.a_tw; % True wind speed
    psi_tw = wind.psi_tw; % True wind direction

    % % Define heading constraint range
    % min_heading = pi/32;
    % max_heading = 31*pi/32;

    % % Apply heading constraint
    % if theta >= min_heading && theta <= max_heading
    %     dtheta = delta_r; % Prevent heading change within the constrained range
    % else
    %     dtheta = 0; % Allow heading change outside the constrained range
    % end

    dtheta = delta_r; % No heading constraint

    % Compute the state derivatives
    dx = v * cos(theta) + v_cx_interp;
    dy = v * sin(theta) + v_cy_interp;
    dv = 0; % Constant velocity

    % Return state derivatives
    dstate = [dx; dy; dtheta; dv];
end