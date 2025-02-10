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

    % Extract environmental conditions
    a_tw = wind.a_tw; % True wind speed
    psi_tw = wind.psi_tw; % True wind direction
    v_cx = currents.v_cx; % Ocean current in x-direction
    v_cy = currents.v_cy; % Ocean current in y-direction

    % --- Compute Apparent Wind (Wind Relative to Boat Motion) ---
    a_aw = sqrt((a_tw * cos(psi_tw - theta) - v)^2 + (a_tw * sin(psi_tw - theta))^2);
    psi_aw = atan2(a_tw * sin(psi_tw - theta), a_tw * cos(psi_tw - theta) - v);

    % --- NACA 0015 Airfoil Calculations ---
    % Load airfoil data only once
    persistent alpha_data Cl_data Cd_data
    if isempty(alpha_data)
        data = readtable('xf-naca0015-il-50000.csv');
        alpha_data = data.Alpha;
        Cl_data = data.Cl;
        Cd_data = data.Cd;
    end

    % Compute Angle of Attack (AoA) based on fixed sail
    alpha_radians = psi_aw - delta_s;
    alpha_degs = alpha_radians * (180/pi); % Convert to degrees

    % Clamp AoA to prevent extreme Cl/Cd values
    alpha_degs = max(min(alpha_degs, 20), -20); 

    % % Interpolate Cl and Cd from airfoil data
    % Cl = interp1(alpha_data, Cl_data, alpha_degs, 'linear', 'extrap');
    % Cd = interp1(alpha_data, Cd_data, alpha_degs, 'linear', 'extrap'); 

    % Ensure interpolation is only performed with numeric data
    if isa(alpha_degs, 'sym')
        Cl = sym('Cl'); % Placeholder symbolic variable
        Cd = sym('Cd'); % Placeholder symbolic variable
    else
        Cl = interp1(alpha_data, Cl_data, alpha_degs, 'linear', 'extrap');
        Cd = interp1(alpha_data, Cd_data, alpha_degs, 'linear', 'extrap');
    end

    % Compute Lift (L) and Drag (D) forces from the sail
    L = damping_factor * 0.5 * air_density * a_aw^2 * sail_area * Cl;
    D = 0.5 * air_density * a_aw^2 * sail_area * Cd;

    % Convert Lift and Drag to boat-aligned components
    lift_angle = psi_aw + pi/2; % Lift is perpendicular to apparent wind
    drag_angle = psi_aw;         % Drag is aligned with apparent wind

    Fx_lift = L * cos(lift_angle);
    Fy_lift = L * sin(lift_angle);

    Fx_drag = D * cos(drag_angle);
    Fy_drag = D * sin(drag_angle);

    % Compute net wind force acting on the boat
    wind_force = [Fx_lift - Fx_drag;
                  Fy_lift - Fy_drag];

    % --- Compute Sail and Rudder Forces ---
    g_s = wind_force(1) * sin(delta_s - psi_aw); % Sail force contribution
    g_r = D * sin(delta_r); % Rudder force contribution

    % --- State-space representation ---
    dx = v * cos(theta) + p1 * a_tw * cos(psi_tw) + v_cx; % x-position
    dy = v * sin(theta) + p1 * a_tw * sin(psi_tw) + v_cy; % y-position
    dtheta = heading_damping * omega; % Smoothed heading change
    dv = (g_s - g_r * p11 - p2 * v^2) / p9; % Boat velocity change
    domega = (g_s * (p6 - p7 * cos(delta_s)) - g_r * p8 * cos(delta_r) - p3 * omega * v) / p10; % Angular acceleration

    % --- Return state derivatives ---
    dstate = [dx; dy; dtheta; dv; domega];
end