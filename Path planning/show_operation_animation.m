function show_operation_animation(position, operation_type, delay_time)
%% Show Operation Animation
% Display operation animation at specified position
% Input:
%   position - Operation position [x, y]
%   operation_type - Operation type
%   delay_time - Delay time

    % Create or get graphic object
    h = findobj('Tag', 'operation_text');
    if isempty(h)
        h = text(position(1), position(2) + 30, '', ...
                 'Tag', 'operation_text', ...
                 'FontSize', 12, ...
                 'FontWeight', 'bold', ...
                 'Color', [0.8 0.2 0.2], ...
                 'HorizontalAlignment', 'center');
    else
        set(h, 'Position', [position(1), position(2) + 30, 0]);
    end
    
    % Show operation info
    status_text = sprintf('[%s] Time: %.1fs', operation_type, delay_time);
    set(h, 'String', status_text, 'Visible', 'on');
    
    % Draw operation indicator circles (pulse effect)
    theta = linspace(0, 2*pi, 50);
    for i = 1:3
        radius = 15 + i * 5;
        x_circle = position(1) + radius * cos(theta);
        y_circle = position(2) + radius * sin(theta);
        
        h_circle = plot(x_circle, y_circle, '--', ...
                       'Color', [0.8 0.4 0.2], ...
                       'LineWidth', 1.5);
        
        pause(delay_time / 6);
        delete(h_circle);
    end
    
    % Hide operation text
    set(h, 'Visible', 'off');
    
end
