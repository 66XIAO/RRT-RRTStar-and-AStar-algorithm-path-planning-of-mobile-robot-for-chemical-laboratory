function [path, success] = rrt_plan(start, goal, obstacles, step_size, max_iterations, robot_radius)
%% RRT Path Planning Algorithm
% Input:
%   start - Start coordinates [x, y]
%   goal - Goal coordinates [x, y]
%   obstacles - Obstacle matrix [x, y, width, height]
%   step_size - Expansion step size
%   max_iterations - Maximum iterations
%   robot_radius - Robot radius (for collision detection)
% Output:
%   path - Planned path
%   success - Whether path was found successfully

    % Ensure start and goal are row vectors
    start = start(:)';
    goal = goal(:)';
    
    % Initialize tree structure
    tree.nodes = start;  % Node coordinates (N x 2 matrix)
    tree.parents = 0;    % Parent node index (0 means root)
    
    % Goal threshold
    goal_threshold = step_size;
    
    success = false;
    goal_index = -1;
    
    for iter = 1:max_iterations
        % Random sampling (with some probability directly sample goal)
        if rand() < 0.1
            random_point = goal;
        else
            random_point = [rand() * 1000, rand() * 800];
        end
        
        % Find nearest node in tree
        % Ensure proper matrix subtraction
        diff_matrix = tree.nodes - repmat(random_point, size(tree.nodes, 1), 1);
        distances = sqrt(sum(diff_matrix.^2, 2));
        [~, nearest_index] = min(distances);
        nearest_node = tree.nodes(nearest_index, :);
        
        % Calculate new node position
        direction = random_point - nearest_node;
        distance = norm(direction);
        
        if distance > 0
            direction = direction / distance;
            new_node = nearest_node + direction * min(step_size, distance);
        else
            continue;
        end
        
        % Check if new node is valid (within bounds and no collision)
        if is_valid_node(new_node, obstacles, robot_radius)
            % Check if edge is valid
            if is_valid_edge(nearest_node, new_node, obstacles, robot_radius)
                % Add to tree
                tree.nodes = [tree.nodes; new_node];
                tree.parents = [tree.parents; nearest_index];
                
                % Check if reached goal
                if norm(new_node - goal) < goal_threshold
                    goal_index = size(tree.nodes, 1);
                    success = true;
                    break;
                end
            end
        end
    end
    
    % Extract path
    if success
        path = extract_path(tree, goal_index);
        % Add goal point
        path = [path; goal];
    else
        % If no complete path found, return closest path to goal
        diff_matrix = tree.nodes - repmat(goal, size(tree.nodes, 1), 1);
        distances_to_goal = sqrt(sum(diff_matrix.^2, 2));
        [~, closest_index] = min(distances_to_goal);
        path = extract_path(tree, closest_index);
        fprintf('  Warning: No complete path found, returning closest path\n');
    end
    
end

%% Check if node is valid
function valid = is_valid_node(node, obstacles, robot_radius)
    valid = true;
    
    % Check bounds
    if node(1) < robot_radius || node(1) > 1000 - robot_radius || ...
       node(2) < robot_radius || node(2) > 800 - robot_radius
        valid = false;
        return;
    end
    
    % Check obstacle collision
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1) - robot_radius;
        y = obstacles(i, 2) - robot_radius;
        w = obstacles(i, 3) + 2 * robot_radius;
        h = obstacles(i, 4) + 2 * robot_radius;
        
        if node(1) >= x && node(1) <= x + w && ...
           node(2) >= y && node(2) <= y + h
            valid = false;
            return;
        end
    end
end

%% Check if edge is valid
function valid = is_valid_edge(node1, node2, obstacles, robot_radius)
    valid = true;
    
    % Sample multiple points along edge for checking
    num_samples = max(ceil(norm(node2 - node1) / 5), 5);
    
    for t = linspace(0, 1, num_samples)
        point = node1 + t * (node2 - node1);
        if ~is_valid_node(point, obstacles, robot_radius)
            valid = false;
            return;
        end
    end
end

%% Extract path from tree
function path = extract_path(tree, end_index)
    path = [];
    current = end_index;
    
    while current ~= 0
        path = [tree.nodes(current, :); path];
        current = tree.parents(current);
    end
end
