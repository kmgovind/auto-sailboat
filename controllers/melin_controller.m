function controls = melin_controller(current_position, current_heading, boat_speed, wind_speed, wind_direction, waypoints, waypoint_index)
% MELIN_CONTROLLER A "sin-controller" for heading control, inspired by Melin (Equation 3.6).
%
% Inputs:
%   current_position  : [x, y] current boat position
%   current_heading   : current boat heading (radians)
%   boat_speed        : current boat speed (m/s)
%   wind_speed        : true wind speed (m/s)
%   wind_direction    : true wind direction (radians), 0 = East, pi/2 = North
%   waypoints         : Nx2 matrix of [x, y] waypoint positions
%   waypoint_index    : current active waypoint index
%
% Output (struct):
%   controls.delta_r  : rudder angle (radians)
%   controls.delta_s  : sail angle (radians)
%
% Additional logic:
%   - If the waypoint is reached, increments waypoint_index
%   - Ignores no-go angle or tacking logic here for brevity
%   - Sin-controller saturates the rudder if cos(error) < 0

% 1) Define Gains / Limits
max_rudder_angle = 30 * pi/180;      % 30 degrees
max_sail_angle   = 60 * pi/180;      % 60 degrees open
min_sail_angle   = 40 * pi/180;      % Fully closed (40 degrees)

% 2) Compute Desired Heading from Current Waypoint
% Get active waypoint
if waypoint_index > size(waypoints, 1)
% If we've exhausted the waypoint list, just hold heading
desired_heading = current_heading;
else
desired_waypoint = waypoints(waypoint_index, :);
dx = desired_waypoint(1) - current_position(1);
dy = desired_waypoint(2) - current_position(2);
desired_heading = atan2(dy, dx);
end

% Check if we've reached the waypoint
distance_to_waypoint = norm(desired_waypoint - current_position);
if distance_to_waypoint < 5.0  % threshold for "waypoint reached" units: meters
waypoint_index = min(waypoint_index + 1, size(waypoints, 1));
end

% 3) Heading Error for Sin-Controller
heading_error = wrapToPi(desired_heading - current_heading);

% 4) Sin-Controller for Rudder (Melin eqn. 3.6: usin(k) = sin(e(k))δr,max) 
% if cos(e) < 0, saturate rudder to ±max_rudder_angle
if cos(heading_error) < 0
delta_r = sign(sin(heading_error)) * max_rudder_angle;
else
% Otherwise scale by sin(e)
delta_r = sin(heading_error) * max_rudder_angle;
end

% 5) Compute Apparent Wind Angle (AWA) for Sail Trim
% A simple approach:
% AWA = angle between wind and boat heading, adjusted for boat_speed
% This is the same formula often used in your "simple_controller"
AWA = atan2(sin(wind_direction - current_heading), cos(wind_direction - current_heading) - boat_speed / wind_speed );

% 6) Simple "optimal trim" for the Sail
% A typical approach is a "cos" shape around AWA=0 => close-hauled
delta_s = (pi/2) * cos(AWA);   % from the paper: eqn (3.3) style 

% Limit sail angle
delta_s = max(min(delta_s, max_sail_angle), min_sail_angle);

% 7) Output
controls = struct('delta_r', delta_r, 'delta_s', delta_s, ...
'waypoint_index', waypoint_index);
end