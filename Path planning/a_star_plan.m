function [path, success] = a_star_plan(start, goal, obstacles, robot_radius, grid_resolution)
%% A* Path Planning Algorithm
% Input:
%   start - Start coordinates [x, y]
%   goal - Goal coordinates [x, y]
%   obstacles - Obstacle matrix [x, y, width, height]
%   robot_radius - Robot radius (for collision detection)
%   grid_resolution - Grid resolution (default: 10)
% Output:
%   path - Planned path
%   success - Whether path was found successfully

    if nargin < 5
        grid_resolution = 10;  % Default grid resolution (cm)
    end
    
    % Ensure start and goal are row vectors
    start = round(start(:)');
    goal = round(goal(:)');
    
    % Map dimensions
    map_width = 1000;
    map_height = 800;
    
    % Create grid
    grid_x = 1:grid_resolution:map_width;
    grid_y = 1:grid_resolution:map_height;
    
    % Convert coordinates to grid indices
    start_idx = [find_closest(grid_x, start(1)), find_closest(grid_y, start(2))];
    goal_idx = [find_closest(grid_x, goal(1)), find_closest(grid_y, goal(2))];
    
    % Create obstacle grid
    obstacle_grid = create_obstacle_grid(obstacles, grid_x, grid_y, robot_radius);
    
    % Check if start or goal is in obstacle
    if obstacle_grid(start_idx(2), start_idx(1)) || obstacle_grid(goal_idx(2), goal_idx(1))
        fprintf('  Warning: Start or goal is in obstacle!\n');
        path = [start; goal];
        success = false;
        return;
    end
    
    % A* algorithm
    open_set = start_idx;  % Nodes to explore [x_idx, y_idx]
    closed_set = [];       % Explored nodes
    
    % Cost matrices
    g_score = inf(length(grid_y), length(grid_x));
    g_score(start_idx(2), start_idx(1)) = 0;
    
    f_score = inf(length(grid_y), length(grid_x));
    f_score(start_idx(2), start_idx(1)) = heuristic(start_idx, goal_idx);
    
    % Parent tracking
    parents = cell(length(grid_y), length(grid_x));
    
    % Directions (8-connected grid)
    directions = [
        1, 0;   % Right
        -1, 0;  % Left
        0, 1;   % Up
        0, -1;  % Down
        1, 1;   % Up-Right
        1, -1;  % Down-Right
        -1, 1;  % Up-Left
        -1, -1  % Down-Left
    ];
    
    success = false;
    
    while ~isempty(open_set)
        % Find node with lowest f_score in open_set
        min_f = inf;
        min_idx = 1;
        for i = 1:size(open_set, 1)
            x = open_set(i, 1);
            y = open_set(i, 2);
            if f_score(y, x) < min_f
                min_f = f_score(y, x);
                min_idx = i;
            end
        end
        
        current = open_set(min_idx, :);
        
        % Check if reached goal
        if current(1) == goal_idx(1) && current(2) == goal_idx(2)
            success = true;
            break;
        end
        
        % Move current from open to closed set
        open_set(min_idx, :) = [];
        closed_set = [closed_set; current];
        
        % Explore neighbors
        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);
            
            % Check bounds
            if neighbor(1) < 1 || neighbor(1) > length(grid_x) || ...
               neighbor(2) < 1 || neighbor(2) > length(grid_y)
                continue;
            end
            
            % Check if in closed set
            if is_in_set(neighbor, closed_set)
                continue;
            end
            
            % Check if obstacle
            if obstacle_grid(neighbor(2), neighbor(1))
                continue;
            end
            
            % Calculate tentative g_score
            if directions(i, 1) == 0 || directions(i, 2) == 0
                tentative_g = g_score(current(2), current(1)) + grid_resolution;
            else
                tentative_g = g_score(current(2), current(1)) + grid_resolution * sqrt(2);
            end
            
            % Check if neighbor is in open set
            in_open = is_in_set(neighbor, open_set);
            
            if ~in_open || tentative_g < g_score(neighbor(2), neighbor(1))
                parents{neighbor(2), neighbor(1)} = current;
                g_score(neighbor(2), neighbor(1)) = tentative_g;
                f_score(neighbor(2), neighbor(1)) = tentative_g + heuristic(neighbor, goal_idx);
                
                if ~in_open
                    open_set = [open_set; neighbor];
                end
            end
        end
    end
    
    % Extract path
    if success
        path = extract_path_a_star(parents, goal_idx, grid_x, grid_y);
        path = [start; path; goal];  % Add exact start and goal
    else
        fprintf('  Warning: A* could not find path!\n');
        path = [start; goal];
    end
    
end

%% Find closest value in array
function idx = find_closest(array, value)
    [~, idx] = min(abs(array - value));
end

%% Create obstacle grid
function grid = create_obstacle_grid(obstacles, grid_x, grid_y, robot_radius)
    grid = false(length(grid_y), length(grid_x));
    
    for i = 1:size(obstacles, 1)
        x_min = obstacles(i, 1) - robot_radius;
        y_min = obstacles(i, 2) - robot_radius;
        x_max = obstacles(i, 1) + obstacles(i, 3) + robot_radius;
        y_max = obstacles(i, 2) + obstacles(i, 4) + robot_radius;
        
        x_min_idx = find_closest(grid_x, x_min);
        x_max_idx = find_closest(grid_x, x_max);
        y_min_idx = find_closest(grid_y, y_min);
        y_max_idx = find_closest(grid_y, y_max);
        
        grid(y_min_idx:y_max_idx, x_min_idx:x_max_idx) = true;
    end
end

%% Check if node is in set
function result = is_in_set(node, set)
    result = false;
    for i = 1:size(set, 1)
        if set(i, 1) == node(1) && set(i, 2) == node(2)
            result = true;
            return;
        end
    end
end

%% Heuristic function (Euclidean distance)
function h = heuristic(node, goal)
    dx = abs(node(1) - goal(1));
    dy = abs(node(2) - goal(2));
    h = sqrt(dx^2 + dy^2);
end

%% Extract path from A* result
function path = extract_path_a_star(parents, goal_idx, grid_x, grid_y)
    path = [];
    current = goal_idx;
    
    while ~isempty(current)
        x = grid_x(current(1));
        y = grid_y(current(2));
        path = [x, y; path];
        
        parent = parents{current(2), current(1)};
        if isempty(parent)
            break;
        end
        current = parent;
    end
end
