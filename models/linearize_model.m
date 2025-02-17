% function [A, B] = linearize_model(state_eq, control_eq, params, wind, currents)
%     % Define symbolic variables
%     syms x y theta v omega delta_s delta_r real
% 
%     % Define state and control vectors
%     state = [x; y; theta; v; omega];
%     % controls = [delta_s; delta_r];
% 
%     % Compute state derivatives using the sailboat dynamics function
%     dstate = sailboat_dynamics(0, state, params, struct('delta_s', delta_s, 'delta_r', delta_r), wind, currents);
% 
%     % Compute the Jacobian matrices
%     A = jacobian(dstate, state); % Partial derivatives of dynamics w.r.t. state
%     % B = jacobian(dstate, controls); % Partial derivatives of dynamics w.r.t. control inputs
% 
%     % Convert symbolic expressions to numeric before calling sailboat_dynamics
%     state_eq = double(state_eq);
%     control_eq.delta_s = double(control_eq.delta_s);
%     control_eq.delta_r = double(control_eq.delta_r);
% 
%     % Evaluate at equilibrium state and control inputs
%     A = subs(A, state, state_eq);
%     display(A);
%     % B = subs(B, state, state_eq);
%     % B = subs(B, controls, control_eq);
% 
%     % Convert symbolic expressions to numeric values
%     A = double(A);
%     B = 0;
% end

function [A, B] = linearize_model(state_eq, control_eq, params, wind, currents)
    % Small perturbation value for numerical differentiation
    epsilon = 1e-6;
    
    % State and control sizes
    num_states = length(state_eq);
    num_controls = length(fieldnames(control_eq));

    % Initialize A and B matrices
    A = zeros(num_states, num_states);
    B = zeros(num_states, num_controls);

    % Compute A matrix using finite difference
    for i = 1:num_states
        perturb = zeros(num_states, 1);
        perturb(i) = epsilon;
        
        % Compute finite difference approximation for state derivative
        dstate_plus = sailboat_dynamics(0, state_eq + perturb, params, control_eq, wind, currents);
        dstate_minus = sailboat_dynamics(0, state_eq - perturb, params, control_eq, wind, currents);
        
        A(:, i) = (dstate_plus - dstate_minus) / (2 * epsilon);
    end

    % Compute B matrix using finite difference
    control_vec = [control_eq.delta_s; control_eq.delta_r];
    for i = 1:num_controls
        perturb = zeros(num_controls, 1);
        perturb(i) = epsilon;
        
        % Convert perturbed controls into struct format
        control_perturb = struct('delta_s', control_vec(1) + perturb(1), 'delta_r', control_vec(2) + perturb(2));

        % Compute finite difference approximation for control derivative
        dstate_plus = sailboat_dynamics(0, state_eq, params, control_perturb, wind, currents);
        control_perturb = struct('delta_s', control_vec(1) - perturb(1), 'delta_r', control_vec(2) - perturb(2));
        dstate_minus = sailboat_dynamics(0, state_eq, params, control_perturb, wind, currents);
        
        B(:, i) = (dstate_plus - dstate_minus) / (2 * epsilon);
    end
end
