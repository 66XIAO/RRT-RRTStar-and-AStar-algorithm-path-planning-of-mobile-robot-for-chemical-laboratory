%% Chemical Laboratory Robot Path Planning System
% Function: Robot automatic path planning in chemical laboratory
% Author: AI Assistant
% Date: 2026-05-13
% 
% Algorithm Selection:
%   planner_type = 'RRT'     - RRT (Rapidly-exploring Random Tree)
%   planner_type = 'RRTStar' - RRT* (RRT with optimization)
%   planner_type = 'AStar'   - A* (A-Star grid-based)
% 
% Usage:
%   lab_robot_rrt('RRT')      - Use RRT algorithm
%   lab_robot_rrt('RRTStar')  - Use RRT* algorithm
%   lab_robot_rrt('AStar')    - Use A* algorithm
%   lab_robot_rrt()           - Default: RRT

clear; clc; close all;

%% Select Path Planning Algorithm
% Change this line to switch algorithms:
% 'RRT' = RRT algorithm
% 'RRTStar' = RRT* algorithm  
% 'AStar' = A* algorithm
planner_type = 'RRTStar';

%% Initialize Laboratory Environment
fprintf('=== Chemical Laboratory Robot Path Planning System ===\n');
fprintf('Path Planning Algorithm: %s\n', planner_type);
fprintf('Initializing laboratory environment...\n');

% Create laboratory map
[obstacles, labels, interaction_points, robot_start] = create_lab_map();

% Display laboratory map
figure('Name', 'Chemical Laboratory Path Planning', 'Position', [100 100 1200 800]);
plot_lab_map(obstacles, labels, interaction_points, robot_start);

%% Define Task Sequence
task_sequence = {
    'reagent_cabinet',    'Get Reagent';
    'balance',            'Measure Reagent';
    'workbench1',         'Place Reagent';
    'utensil_cabinet',    'Get Instrument';
    'workbench1',         'Conduct Experiment';
    'fume_hood',          'Transfer Product';
    'balance',            'Measure Product';
    'storage',            'Store Product';
    'workbench1',         'Remove Instrument';
    'sink',               'Clean Instrument';
    'utensil_cabinet',    'Store Instrument';
    'entrance',           'Experiment Complete'
};

fprintf('\nExperiment Task Sequence:\n');
for i = 1:size(task_sequence, 1)
    fprintf('%d. %s - %s\n', i, task_sequence{i,1}, task_sequence{i,2});
end

%% Wait for User Confirmation
fprintf('\nPress any key to start experiment...\n');
pause;

%% Algorithm Parameters
step_size = 8;          % Step size (for RRT/RRT*)
max_iterations = 5000;  % Maximum iterations (for RRT/RRT*)
robot_radius = 5;       % Robot radius (non-point model)
grid_resolution = 10;   % Grid resolution (for A*)
rewire_radius = 50;     % Rewire radius (for RRT*)

%% Execute Path Planning
full_path = robot_start;  % Complete path
waypoints = robot_start;  % Waypoints

for i = 1:size(task_sequence, 1)
    target_name = task_sequence{i, 1};
    task_desc = task_sequence{i, 2};
    
    % Get target interaction point
    target_point = interaction_points.(target_name);
    
    fprintf('\n[%d/%d] Moving to %s (%s)...\n', i, size(task_sequence,1), target_name, task_desc);
    
    % Use selected algorithm to plan path
    current_pos = full_path(end, :);
    
    switch upper(planner_type)
        case 'RRT'
            [path_segment, success] = rrt_plan(current_pos, target_point, obstacles, step_size, max_iterations, robot_radius);
        case 'RRTSTAR'
            [path_segment, success] = rrt_star_plan(current_pos, target_point, obstacles, step_size, max_iterations, robot_radius, rewire_radius);
        case 'ASTAR'
            [path_segment, success] = a_star_plan(current_pos, target_point, obstacles, robot_radius, grid_resolution);
        otherwise
            fprintf('  Warning: Unknown planner type "%s", using RRT\n', planner_type);
            [path_segment, success] = rrt_plan(current_pos, target_point, obstacles, step_size, max_iterations, robot_radius);
    end
    
    if success
        % Add path segment to complete path
        full_path = [full_path; path_segment(2:end, :)];
        waypoints = [waypoints; target_point];
        
        % Draw path
        plot(path_segment(:,1), path_segment(:,2), 'b-', 'LineWidth', 2);
        plot(target_point(1), target_point(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        % Simulate delay (experiment operation time)
        delay_time = get_operation_delay(task_desc);
        fprintf('  Arrived at %s, performing operation: %s (delay %.1f seconds)\n', target_name, task_desc, delay_time);
        
        % Show operation animation
        show_operation_animation(target_point, task_desc, delay_time);
        
        pause(0.5);
    else
        fprintf('  Warning: Cannot find path to %s!\n', target_name);
    end
end

%% Path Smoothing
fprintf('\nOptimizing path smoothness...\n');
smoothed_path = smooth_path(full_path, obstacles, robot_radius);

%% Draw Final Path
plot(smoothed_path(:,1), smoothed_path(:,2), 'g-', 'LineWidth', 3);

%% Robot Movement Animation
fprintf('Playing robot movement animation...\n');
animate_robot(smoothed_path, obstacles, labels, interaction_points, robot_radius);

fprintf('\n=== Experiment Complete! Robot returned to initial position ===\n');
