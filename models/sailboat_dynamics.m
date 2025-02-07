function dstate = sailboat_dynamics(t, state, params, controls, wind, currents)
    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3); % Boat heading
    v = state(4);     % Boat velocity
    omega = state(5); % Angular velocity (yaw rate)

    % Constants
    air_density = 1.225; % kg/m^3
    sail_area = 0.5; % m^2 (adjust as needed)
    damping_factor = 0.7; % Damping factor to stabilize forces
    heading_damping = 0.5; % Reduce heading oscillations

    % Extract model parameters
    p1 = params.p1;
    p2 = params.p2;
    p3 = params.p3;
    p4 = params.p4; % Sail force scaling
    p5 = params.p5; % Rudder force scaling
    p6 = params.p6;
    p7 = params.p7;
    p8 = params.p8;
    p9 = params.p9;
    p10 = params.p10;
    p11 = params.p11;

    % Extract control inputs
    delta_s = controls.delta_s; % Fixed sail angle
    delta_r = controls.delta_r; % Fixed rudder angle

    % Interpolate wind speed and direction at the current time
    a_tw = interp1(wind.t, wind.a_tw, t, 'linear', 'extrap');
    psi_tw = interp1(wind.t, wind.psi_tw, t, 'linear', 'extrap');

    % --- Compute Apparent Wind (Wind Relative to Boat Motion) ---
    a_aw = sqrt((a_tw * cos(psi_tw - theta) - v)^2 + (a_tw * sin(psi_tw - theta))^2);
    psi_aw = atan2(a_tw * sin(psi_tw - theta), a_tw * cos(psi_tw - theta) - v);

    % --- NACA 0015 Airfoil Calculations ---
    persistent alpha_data Cl_data Cd_data
    if isempty(alpha_data)
        data = readtable('xf-naca0015-il-50000.csv');
        alpha_data = data.Alpha;
        Cl_data = data.Cl;
        Cd_data = data.Cd;
    end

    % Compute Angle of Attack (AoA)
    alpha_radians = psi_aw - delta_s;
    alpha_degs = alpha_radians * (180/pi);
    alpha_degs = max(min(alpha_degs, 20), -20); % Clamp AoA

    % Interpolate Cl and Cd
    Cl = interp1(alpha_data, Cl_data, alpha_degs, 'linear', 'extrap');
    Cd = interp1(alpha_data, Cd_data, alpha_degs, 'linear', 'extrap');

    % Compute Lift (L) and Drag (D) forces
    L = damping_factor * 0.5 * air_density * a_aw^2 * sail_area * Cl;
    D = 0.5 * air_density * a_aw^2 * sail_area * Cd;

    % Convert forces to boat-aligned components
    lift_angle = psi_aw + pi/2;
    drag_angle = psi_aw;

    Fx_lift = L * cos(lift_angle);
    Fy_lift = L * sin(lift_angle);
    Fx_drag = D * cos(drag_angle);
    Fy_drag = D * sin(drag_angle);

    wind_force = [Fx_lift - Fx_drag; Fy_lift - Fy_drag];

    dx = v * cos(theta) + p1 * a_tw * cos(psi_tw) + currents.v_cx;
    dy = v * sin(theta) + p1 * a_tw * sin(psi_tw) + currents.v_cy;
    dtheta = heading_damping * omega;
    dv = (wind_force(1) - p2 * v^2) / p9;
    domega = (wind_force(2) - p3 * omega * v) / p10;

    dstate = [dx; dy; dtheta; dv; domega];
end