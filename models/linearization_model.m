syms x y heading velocity sail_angle control_input wind_strength wind_direction current_strength current_direction

% Define the state and input vectors
state = [x; y; heading; velocity; sail_angle];
control_input_sym = control_input;

% Define the high fidelity model equations
[state_dot] = high_fidelity_model(state, control_input_sym, [wind_strength; wind_direction], [current_strength; current_direction]);

% Define the equilibrium conditions (state_dot = 0)
eq_conditions = state_dot == 0;

% Solve for equilibrium points
equilibrium_points = solve(eq_conditions, [x, y, heading, velocity, sail_angle]);

% Compute Jacobian matrices
A = jacobian(state_dot, state);
B = jacobian(state_dot, control_input_sym);

% Evaluate Jacobians at the equilibrium point
A_eq = subs(A, state, [equilibrium_points.x, equilibrium_points.y, equilibrium_points.heading, equilibrium_points.velocity, equilibrium_points.sail_angle]);
B_eq = subs(B, state, [equilibrium_points.x, equilibrium_points.y, equilibrium_points.heading, equilibrium_points.velocity, equilibrium_points.sail_angle]);

disp('Equilibrium Points:');
disp(equilibrium_points);

disp('Jacobian Matrix A:');
disp(A_eq);

disp('Jacobian Matrix B:');
disp(B_eq);
