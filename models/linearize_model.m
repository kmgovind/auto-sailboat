function [A, B] = linearize_model(state_eq, control_eq, params, wind, currents)
    % Define symbolic variables
    syms x y theta v omega delta_s delta_r real
    
    % Define state and control vectors
    state = [x; y; theta; v; omega];
    controls = [delta_s; delta_r];
    
    % Compute state derivatives using the sailboat dynamics function
    dstate = sailboat_dynamics(0, state, params, struct('delta_s', delta_s, 'delta_r', delta_r), wind, currents);
    
    % Compute the Jacobian matrices
    A = jacobian(dstate, state); % Partial derivatives of dynamics w.r.t. state
    B = jacobian(dstate, controls); % Partial derivatives of dynamics w.r.t. control inputs
    
    % Evaluate at equilibrium state and control inputs
    A = subs(A, state, state_eq);
    B = subs(B, state, state_eq);
    B = subs(B, controls, control_eq);
    
    % Convert symbolic expressions to numeric values
    A = double(A);
    B = double(B);
end