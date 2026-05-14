function delay_time = get_operation_delay(operation_type)
%% Get operation delay time
% Return corresponding delay time based on operation type (seconds)
% Input:
%   operation_type - Operation type description
% Output:
%   delay_time - Delay time (seconds)

    switch operation_type
        case 'Get Reagent'
            delay_time = 3.0;  % Open cabinet, get reagent, close cabinet
        case 'Measure Reagent'
            delay_time = 5.0;  % Place reagent, measure, record data
        case 'Place Reagent'
            delay_time = 2.0;  % Place reagent on workbench
        case 'Get Instrument'
            delay_time = 4.0;  % Open cabinet, get instrument, close cabinet
        case 'Conduct Experiment'
            delay_time = 10.0; % Experiment operation time
        case 'Transfer Product'
            delay_time = 4.0;  % Take out product, transfer to storage
        case 'Measure Product'
            delay_time = 5.0;  % Measure product parameters
        case 'Store Product'
            delay_time = 3.0;  % Store product
        case 'Remove Instrument'
            delay_time = 2.5;  % Remove instrument
        case 'Clean Instrument'
            delay_time = 6.0;  % Clean and cool instrument
        case 'Store Instrument'
            delay_time = 3.0;  % Store instrument in cabinet
        case 'Experiment Complete'
            delay_time = 1.0;  % Experiment complete, return to start
        otherwise
            delay_time = 2.0;  % Default delay
    end
    
    % Simulate delay (can be replaced with actual delay in real use)
    % pause(delay_time);
    
end
