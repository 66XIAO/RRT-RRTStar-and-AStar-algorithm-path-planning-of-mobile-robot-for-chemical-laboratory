function plot_lab_map(obstacles, labels, interaction_points, robot_start)
%% Draw Laboratory Map
% Input:
%   obstacles - Obstacle matrix
%   labels - Label structure
%   interaction_points - Interaction point coordinates
%   robot_start - Robot starting position

    hold on;
    grid on;
    axis equal;
    xlim([0 1000]);
    ylim([0 800]);
    xlabel('X (cm)', 'FontSize', 12);
    ylabel('Y (cm)', 'FontSize', 12);
    title('Chemical Lab Robot Path Planning - RRT Algorithm', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Draw obstacles
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1);
        y = obstacles(i, 2);
        w = obstacles(i, 3);
        h = obstacles(i, 4);
        
        if i <= 7  % Lab equipment
            color = [0.8 0.8 0.9];
            edge_color = [0.3 0.3 0.6];
        else  % Walls
            color = [0.4 0.4 0.4];
            edge_color = [0.2 0.2 0.2];
        end
        
        rectangle('Position', [x, y, w, h], ...
                  'FaceColor', color, ...
                  'EdgeColor', edge_color, ...
                  'LineWidth', 2);
    end
    
    % Draw labels (display only, not obstacles)
    label_fields = fieldnames(labels);
    colors = {[0.2 0.2 0.8], [0.8 0.2 0.2], [0.2 0.6 0.2], [0.6 0.2 0.6], ...
              [0.8 0.6 0.2], [0.2 0.6 0.8], [0.8 0.4 0.4], [0.4 0.4 0.8], [0.2 0.8 0.4]};
    
    for i = 1:length(label_fields)
        label_struct = labels.(label_fields{i});
        x = label_struct.x;
        y = label_struct.y;
        text_str = label_struct.text;
        
        text(x, y, text_str, ...
             'FontSize', 10, ...
             'FontWeight', 'bold', ...
             'Color', colors{mod(i-1, length(colors)) + 1}, ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle');
    end
    
    % Draw interaction points
    interaction_names = fieldnames(interaction_points);
    for i = 1:length(interaction_names)
        point = interaction_points.(interaction_names{i});
        plot(point(1), point(2), 'g*', 'MarkerSize', 10);
    end
    
    % Draw robot starting position
    plot(robot_start(1), robot_start(2), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    text(robot_start(1), robot_start(2) - 20, 'Start', ...
         'FontSize', 10, 'Color', 'r', 'HorizontalAlignment', 'center');
    
end
