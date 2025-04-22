clear;clc;close all;
% Load the data from the .mat file
data = load('controller_and_results.mat');

% Assuming the data contains variables 'time' and 'path'
% Replace 'time' and 'path' with the actual variable names in your .mat file
waypoints = data.waypoints
path = data.path;

% Plot and save trajectory
figure;
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2); hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro--', 'LineWidth', 1.5);
plot(path(1,1), path(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('Sailboat Path through Waypoints');
xlabel('X'); ylabel('Y'); grid on; axis equal;
legend('Trajectory', 'Waypoints', 'Start');
saveas(gcf, fullfile('results', 'ga_trajectory.png'));


% Create and save animation of the boat moving along its path
skip_frames = max(1, floor(size(path, 1) / (15 / 0.1))); % Calculate frame skipping factor
figure;
filename = fullfile('results', 'ga_sailboat_animation.gif');
for i = 1:skip_frames:size(path, 1)
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2); hold on;
    plot(waypoints(:,1), waypoints(:,2), 'ro--', 'LineWidth', 1.5);
    plot(path(1,1), path(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(path(i,1), path(i,2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    title('Sailboat Path through Waypoints');
    xlabel('X'); ylabel('Y'); grid on; axis equal;
    legend('Trajectory', 'Waypoints', 'Start', 'Boat');
    drawnow;

    % Capture the frame
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    % Write to the GIF file
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    hold off;
end
