function [obstacles, labels, interaction_points, robot_start] = create_lab_map()
%% Create Chemical Laboratory Map
% Output:
%   obstacles - Obstacle matrix [x, y, width, height]
%   labels - Label structure
%   interaction_points - Interaction point coordinates
%   robot_start - Robot starting position

    % Laboratory size (1000 x 800)
    lab_width = 1000;
    lab_height = 800;
    
    %% Define obstacles (lab tables, cabinets, etc.)
    % Format: [x, y, width, height]
    obstacles = [
        % Reagent cabinet (top left)
        50, 50, 120, 80;
        
        % Balance (left middle)
        50, 250, 100, 100;
        
        % Utensil cabinet (right side)
        800, 100, 120, 200;
        
        % Workbench 1 - next to utensil cabinet
        650, 150, 120, 80;
        
        % Fume hood (top right)
        800, 400, 150, 120;
        
        % Sink (bottom right)
        750, 650, 100, 80;
        
        % Storage cabinet (below workbench)
        300, 600, 100, 80;
        
        % Central lab table
        300, 300, 200, 150;
        
        % Wall obstacles (boundaries)
        0, 0, 1000, 20;      % Top wall
        0, 780, 1000, 20;    % Bottom wall
        0, 0, 20, 800;       % Left wall
        980, 0, 20, 800;     % Right wall
    ];
    
    %% Define label positions (display only, not obstacles)
    % Format: struct with x, y, text fields
    labels.reagent_cabinet.x = 110;
    labels.reagent_cabinet.y = 90;
    labels.reagent_cabinet.text = 'Reagent';
    
    labels.balance.x = 100;
    labels.balance.y = 300;
    labels.balance.text = 'Balance';
    
    labels.utensil_cabinet.x = 860;
    labels.utensil_cabinet.y = 200;
    labels.utensil_cabinet.text = 'Utensil';
    
    labels.workbench1.x = 710;
    labels.workbench1.y = 190;
    labels.workbench1.text = 'Bench1';
    
    labels.fume_hood.x = 875;
    labels.fume_hood.y = 460;
    labels.fume_hood.text = 'FumeHood';
    
    labels.sink.x = 800;
    labels.sink.y = 690;
    labels.sink.text = 'Sink';
    
    labels.storage.x = 350;
    labels.storage.y = 640;
    labels.storage.text = 'Storage';
    
    labels.central_table.x = 400;
    labels.central_table.y = 375;
    labels.central_table.text = 'Central';
    
    labels.entrance.x = 100;
    labels.entrance.y = 750;
    labels.entrance.text = 'Entrance';
    
    %% Define interaction points (where robot actually arrives)
    % These points are in front of equipment for easy operation
    % IMPORTANT: Points must not be blocked by other obstacles
    interaction_points.reagent_cabinet = [200, 90];       % In front of reagent cabinet
    interaction_points.balance = [200, 300];              % In front of balance
    interaction_points.utensil_cabinet = [750, 350];      % Below utensil cabinet (was blocked by workbench)
    interaction_points.workbench1 = [600, 190];           % Left side of workbench1
    interaction_points.fume_hood = [700, 460];            % In front of fume hood
    interaction_points.sink = [650, 690];                 % In front of sink
    interaction_points.storage = [300, 550];              % Above storage cabinet
    interaction_points.entrance = [100, 700];             % At entrance
    
    %% Robot starting position (lab entrance)
    robot_start = [100, 700];
    
end
