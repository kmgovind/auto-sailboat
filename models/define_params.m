function params = define_params()
    % Define model parameters
    params = struct(...
        'p1', 0.03, ...  % Drift Coefficient [-]
        'p2', 40, ... % Tangential Friction [kgs-1]
        'p3', 6000, ...  % Angular Friction [kgm]
        'p4', 4e50, ...  % Sail Lift [kgs-1]
        'p5', 4e50, ...  % Rudder Lift [kgs-1]
        'p6', 0.5, ...  % Distance to Sail CoE [m]
        'p7', 0.5, ...  % Distance to Mast [m]
        'p8', 2, ...  % Distance to Rudder [m]
        'p9', 300, ...  % Mass of Boat [kg]
        'p10', 400, ... % Moment of Inertia [kgm2]
        'p11', 0.2 ...  % Rudder Break Coefficient [-]
    );
end