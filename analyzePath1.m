function [subjective_moves, turns] = analyzePath1(path, initial_heading)
    % 4: forward
    % 5: left
    % 6: right
    % 7: back
    % Check if initial_heading is provided, otherwise default to 'right'
    if nargin < 2
        initial_heading = 'right';
    end
    
    % Validate initial_heading
    valid_headings = {'right', 'left', 'up', 'down'};
    if ~ismember(initial_heading, valid_headings)
        error('Invalid initial heading. Must be one of: right, left, up, down');
    end
    
    % Initialize variables
    subjective_moves = cell(1, size(path, 1) - 1);
    turns = [];
    current_heading = initial_heading;
    
    % Define direction changes
    direction_changes = containers.Map();
    direction_changes('right') = struct('up', 6, 'down', 5, 'left', 7, 'right', 4);
    direction_changes('left') = struct('up', 5, 'down', 6, 'left', 4, 'right', 7);
    direction_changes('up') = struct('up', 4, 'down', 7, 'left', 6, 'right', 5);
    direction_changes('down') = struct('up', 7, 'down', 4, 'left', 5, 'right', 6);
    
    % Define move numbers
    % move_numbers = struct('forward', 4, 'right', 5, 'left', 6, 'backward', 7);
    move_numbers = struct('forward', 4, 'right', 6, 'left', 5, 'backward', 7);
    
    for i = 2:size(path, 1)
        prev = path(i-1, :);
        curr = path(i, :);
        
        dx = curr(1) - prev(1);
        dy = curr(2) - prev(2);
        
        if dy == 1
            absolute_direction = 'right';
        elseif dy == -1
            absolute_direction = 'left';
        elseif dx == 1
            absolute_direction = 'up';
        elseif dx == -1
            absolute_direction = 'down';
        else
            error('Invalid movement detected');
        end
        
        % Determine subjective move
        subjective_move = direction_changes(current_heading).(absolute_direction);
        subjective_moves{i-1} = subjective_move;
        
        % Update heading if necessary
        if subjective_move ~= move_numbers.forward
            turns = [turns, i-1];
            current_heading = absolute_direction;
        end
    end
end