function animate_robot(path, obstacles, labels, interaction_points, robot_radius)
%% Robot Movement Animation
% Display robot movement animation along path
% Input:
%   path - Path points
%   obstacles - Obstacles
%   labels - Labels
%   interaction_points - Interaction points
%   robot_radius - Robot radius

    % Create robot graphic object
    theta = linspace(0, 2*pi, 30);
    
    % Initialize robot graphic
    robot_body_x = path(1, 1) + robot_radius * cos(theta);
    robot_body_y = path(1, 2) + robot_radius * sin(theta);
    
    h_robot = fill(robot_body_x, robot_body_y, [0.2 0.6 0.8], ...
                   'EdgeColor', [0.1 0.3 0.6], ...
                   'LineWidth', 2, ...
                   'Tag', 'robot');
    
    % Add direction indicator
    h_direction = plot(path(1, 1), path(1, 2), 'w>', ...
                      'MarkerSize', 8, ...
                      'MarkerFaceColor', 'w', ...
                      'Tag', 'robot_direction');
    
    % Animation speed
    speed = 5;  % Distance per frame
    
    % Move along path
    current_idx = 1;
    current_pos = path(1, :);
    
    while current_idx < size(path, 1)
        % Calculate next target point
        target_pos = path(current_idx + 1, :);
        
        % Calculate direction vector
        direction = target_pos - current_pos;
        distance = norm(direction);
        
        if distance > 0
            direction = direction / distance;
            
            % Interpolate movement
            step_distance = min(speed, distance);
            num_steps = ceil(distance / step_distance);
            
            for step = 1:num_steps
                % Update position
                if step == num_steps
                    current_pos = target_pos;
                else
                    current_pos = current_pos + direction * step_distance;
                end
                
                % Update robot graphic
                robot_body_x = current_pos(1) + robot_radius * cos(theta);
                robot_body_y = current_pos(2) + robot_radius * sin(theta);
                
                set(h_robot, 'XData', robot_body_x, 'YData', robot_body_y);
                
                % Update direction indicator
                direction_angle = atan2(direction(2), direction(1));
                dir_x = current_pos(1) + (robot_radius * 0.6) * cos(direction_angle);
                dir_y = current_pos(2) + (robot_radius * 0.6) * sin(direction_angle);
                set(h_direction, 'XData', dir_x, 'YData', dir_y);
                
                % Refresh display
                drawnow limitrate;
                pause(0.02);
            end
        end
        
        current_idx = current_idx + 1;
        if current_idx < size(path, 1)
            current_pos = path(current_idx, :);
        end
    end
    
    % Final position marker
    plot(path(end, 1), path(end, 2), 'g*', 'MarkerSize', 15, 'LineWidth', 2);
    text(path(end, 1), path(end, 2) - 25, 'End', ...
         'FontSize', 10, 'Color', 'g', 'HorizontalAlignment', 'center');
    
    fprintf('Robot animation playback complete\n');
    
end
