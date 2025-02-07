% init_workspace.m
% This script initializes the MATLAB workspace for the robotic sailboat simulation project.
% It adds necessary paths for all directories and sets up sample scripts for running the sailboat models.

% Clear the workspace and command window
clear; clc;

% Define the project root directory
projectRoot = pwd;

% Add paths for all subdirectories
addpath(fullfile(projectRoot, 'models'));
addpath(fullfile(projectRoot, 'environment'));
addpath(fullfile(projectRoot, 'controllers'));
addpath(fullfile(projectRoot, 'simulations'));
addpath(fullfile(projectRoot, 'utils'));
addpath(fullfile(projectRoot, 'results'));

% Display a message indicating successful initialization
disp('Workspace initialized. All necessary paths have been added.');

% Sample script to run a simplified model in a specific environment
disp('Select a script from the `simulations` directory to run a simulation.');