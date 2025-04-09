% filepath: c:\Users\kavin\OneDrive\School\Winter_2025\ROB_599\auto-sailboat\tests\test_calculate_sail_and_rudder_deflections.m
% Test script for calculate_sail_and_rudder_deflections

% Define test parameters
params.max_sail_angle = deg2rad(45); % Maximum sail angle in radians
params.max_rudder_angle = deg2rad(30); % Maximum rudder angle in radians

% Test cases
test_cases = {
    % current_speed, current_direction, wind_speed, wind_direction, desired_heading
    {0.5, deg2rad(90), 5.0, deg2rad(45), deg2rad(0)},  % Case 1: Moderate wind and current
    {1.0, deg2rad(180), 3.0, deg2rad(90), deg2rad(90)}, % Case 2: Strong current, weak wind
    {0.0, deg2rad(0), 10.0, deg2rad(270), deg2rad(180)}, % Case 3: No current, strong wind
    {0.2, deg2rad(45), 2.0, deg2rad(135), deg2rad(90)}, % Case 4: Light wind and current
    {0.8, deg2rad(270), 6.0, deg2rad(0), deg2rad(270)}  % Case 5: Strong wind and current
};

% Run test cases
fprintf('Running tests for calculate_sail_and_rudder_deflections...\n');
for i = 1:length(test_cases)
    % Extract test case inputs
    inputs = test_cases{i};
    current_speed = inputs{1};
    current_direction = inputs{2};
    wind_speed = inputs{3};
    wind_direction = inputs{4};
    desired_heading = inputs{5};

    % Call the function
    [delta_r, delta_s] = calculate_sail_and_rudder_deflections(current_speed, current_direction, wind_speed, wind_direction, desired_heading, params);

    % Display results
    fprintf('Test Case %d:\n', i);
    fprintf('  Current Speed: %.2f m/s, Current Direction: %.2f deg\n', current_speed, rad2deg(current_direction));
    fprintf('  Wind Speed: %.2f m/s, Wind Direction: %.2f deg\n', wind_speed, rad2deg(wind_direction));
    fprintf('  Desired Heading: %.2f deg\n', rad2deg(desired_heading));
    fprintf('  Rudder Deflection (delta_r): %.2f deg\n', rad2deg(delta_r));
    fprintf('  Sail Deflection (delta_s): %.2f deg\n', rad2deg(delta_s));

    % Verify outputs are within limits
    assert(abs(delta_r) <= params.max_rudder_angle, 'Rudder deflection exceeds maximum limit!');
    assert(abs(delta_s) <= params.max_sail_angle, 'Sail deflection exceeds maximum limit!');
    fprintf('  Test Case %d passed.\n\n', i);
end

fprintf('All tests passed successfully.\n');