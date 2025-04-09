function delta_r = lqr_controller(state, waypoint)
    % LQR controller for the Dubins boat model to reach a waypoint in minimum time.
    % Inputs:
    %   state    - Current state [x; y; theta; v]
    %   waypoint - Target waypoint [x_target; y_target]
    % Output:
    %   delta_r  - Rudder angle control input

    % Extract state variables
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);

    % Compute the error to the waypoint
    x_error = waypoint(1) - x;
    y_error = waypoint(2) - y;

    % Desired heading to the waypoint
    desired_heading = atan2(y_error, x_error);

    % Linearize the system around the current state
    if v == 0
        v = 0.1; % Avoid singularity by setting a small velocity
    end
    A = [0, 0, -v * sin(theta), cos(theta);
         0, 0,  v * cos(theta), sin(theta);
         0, 0,  0,              0;
         0, 0,  0,              0];
    B = [0; 0; 1; 0];

    % Define LQR cost matrices
    Q = diag([10, 10, 5, 0]); % Penalize position and heading errors
    R = 1;                    % Penalize control effort

    % Check if the system is controllable
    if rank(ctrb(A, B)) < size(A, 1)
        % Use proportional control as a fallback
        k_p = 1.0;
        heading_error = wrapToPi(desired_heading - theta);
        delta_r = k_p * heading_error;
    else
        % Compute the LQR gain matrix
        try
            K = lqr(A, B, Q, R);
        catch ME
            warning('LQR computation failed: %s', ME.message);
            delta_r = 0; % Default to no control
            return;
        end

        % Define the state error vector
        state_error = [x_error; y_error; wrapToPi(desired_heading - theta); 0];

        % Use LQR control
        delta_r = -K * state_error;
    end

    % Limit the rudder angle to a maximum value
    max_rudder_angle = pi / 6; % 30 degrees
    delta_r = max(min(delta_r, max_rudder_angle), -max_rudder_angle);
end