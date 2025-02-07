function dstate = sailboat_dynamics(t, state, params, controls, wind, currents)
    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    omega = state(5);

    % Extract parameters
    p1 = params.p1;
    p2 = params.p2;
    p3 = params.p3;
    p4 = params.p4;
    p5 = params.p5;
    p6 = params.p6;
    p7 = params.p7;
    p8 = params.p8;
    p9 = params.p9;
    p10 = params.p10;
    p11 = params.p11;

    % Extract control inputs
    delta_s = controls.delta_s;
    delta_r = controls.delta_r;

    % Extract environmental conditions
    a_tw = wind.a_tw;
    psi_tw = wind.psi_tw;
    v_cx = currents.v_cx;
    v_cy = currents.v_cy;

    % Compute apparent wind acceleration and angle
    a_aw = sqrt((a_tw * cos(psi_tw - theta) - v)^2 + (a_tw * sin(psi_tw - theta))^2);
    psi_aw = atan2(a_tw * sin(psi_tw - theta), a_tw * cos(psi_tw - theta) - v);

    % Calculate forces
    g_s = p4 * a_aw * sin(delta_s - psi_aw);
    g_r = p5 * v^2 * sin(delta_r);

    % State-space representation
    dx = v * cos(theta) + p1 * a_tw * cos(psi_tw) + v_cx;
    dy = v * sin(theta) + p1 * a_tw * sin(psi_tw) + v_cy;
    dtheta = omega;
    dv = (g_s - g_r * p11 - p2 * v^2) / p9;
    domega = (g_s * (p6 - p7 * cos(delta_s)) - g_r * p8 * cos(delta_r) - p3 * omega * v) / p10;

    % Return state derivatives
    dstate = [dx; dy; dtheta; dv; domega];
end