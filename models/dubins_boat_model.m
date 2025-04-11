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
    a_tw_current = interp1(wind.t,wind.a_tw,t,'linear','extrap');
    psi_tw_current = interp1(wind.t,wind.psi_tw,t,'linear','extrap');

    air_density = 1.225;
    sail_area = 10;
    persistent alpha_data Cl_data Cd_data
    if isempty(alpha_data)
        data = readtable('xf-naca0015-il-50000.csv');
        alpha_data = data.Alpha;
        Cl_data = data.Cl;
        Cd_data = data.Cd;
    end
    delta_s = delta_r; % Keep sail angle same as rudder angle
    a_aw = sqrt((a_tw_current*cos(psi_tw_current - theta) - v)^2 + (a_tw_current*sin(psi_tw_current - theta))^2);
    psi_aw = atan2(a_tw_current*sin(psi_tw_current - theta),a_tw_current*cos(psi_tw_current - theta) - v);
    alpha_radians = psi_aw - delta_s;
    alpha_degs = alpha_radians*(180/pi);
    alpha_degs = max(min(alpha_degs,20),-20);
    Cl = interp1(alpha_data,Cl_data,alpha_degs,'linear','extrap');
    Cd = interp1(alpha_data,Cd_data,alpha_degs,'linear','extrap');
    L = 0.5*air_density*a_aw^2*sail_area*Cl;
    D = 0.5*air_density*a_aw^2*sail_area*Cd;
    Fx_lift = L*cos(psi_aw + pi/2);
    Fy_lift = L*sin(psi_aw + pi/2);
    Fx_drag = D*cos(psi_aw);
    Fy_drag = D*sin(psi_aw);
    wind_force = [Fx_lift - Fx_drag; Fy_lift - Fy_drag];

    % % Define heading constraint range
    % min_heading = pi/32;
    % max_heading = 31*pi/32;

    % % Apply heading constraint
    % if theta >= min_heading && theta <= max_heading
    %     dtheta = delta_r; % Prevent heading change within the constrained range
    % else
    %     dtheta = 0; % Allow heading change outside the constrained range
    % end

    dtheta = delta_r; % Keep rudder -> heading

    Fx_global = wind_force(1)*cos(theta) - wind_force(2)*sin(theta);
    Fy_global = wind_force(1)*sin(theta) + wind_force(2)*cos(theta);

    m = 50;   % mass in kg
    c = 1;  % linear damping coefficient

    dx = v*cos(theta) + v_cx_interp + Fx_global/m;
    dy = v*sin(theta) + v_cy_interp + Fy_global/m;
    dv = (Fx_global*cos(theta) + Fy_global*sin(theta))/m - c*v;

    % Return state derivatives
    dstate = [dx; dy; dtheta; dv];
end