function [path, success] = findPathWHCA(grid, start, goal, other_robots, other_paths)
    window_size = 4;
    reservations = struct('time', {}, 'pos', {}, 'robot', {});
    
    % Add current positions and immediate next positions of other robots
    for i = 1:size(other_robots,1)
        if ~isempty(other_robots(i,:))
            % Reserve current position
            res.time = 1;
            res.pos = other_robots(i,:);
            res.robot = i;
            reservations(end+1) = res;
            
            % Reserve next position if available
            if ~isempty(other_paths{i}) && size(other_paths{i},1) > 1
                res.time = 2;
                res.pos = other_paths{i}(2,:);
                res.robot = i;
                reservations(end+1) = res;
            end
        end
    end
    
    % Add future positions from other robots' paths
    for i = 1:length(other_paths)
        if ~isempty(other_paths{i})
            for t = 1:size(other_paths{i}, 1)
                res.time = t;
                res.pos = other_paths{i}(t,:);
                res.robot = i;
                reservations(end+1) = res;
                
                % Also reserve adjacent cells for one timestep to prevent swapping
                if t > 1
                    prev_pos = other_paths{i}(t-1,:);
                    % Calculate movement vector
                    move_vector = res.pos - prev_pos;
                    if any(move_vector ~= 0)  % If robot is moving
                        % Reserve positions adjacent to movement path
                        adjacent_positions = getAdjacentPositions(prev_pos, res.pos);
                        for adj_pos = adjacent_positions'
                            if all(adj_pos >= 1) && all(adj_pos <= size(grid))
                                res_adj = struct('time', t, 'pos', adj_pos', 'robot', i);
                                reservations(end+1) = res_adj;
                            end
                        end
                    end
                end
            end
        end
    end
    
    % Try direct path first
    [path, found] = findWindowedPath(grid, start, goal, reservations, window_size);
    
    if ~found
        % Try with increased window size
        [path, found] = findWindowedPath(grid, start, goal, reservations, window_size * 2);
    end
    
    if ~found
        % If still no path, try temporary waypoint
        temp_dest = findTemporaryDestination(grid, start, goal, reservations);
        [path, found] = findWindowedPath(grid, start, temp_dest, reservations, window_size);
    end
    
    if ~found
        path = start;  % Return starting position if no path found
        success = false;
    else
        success = true;
    end
end

function adjacent_positions = getAdjacentPositions(pos1, pos2)
    % Get positions adjacent to the movement path between pos1 and pos2
    move_vector = pos2 - pos1;
    if move_vector(1) ~= 0  % Vertical movement
        adjacent_positions = [pos1(1), pos1(2)-1; pos1(1), pos1(2)+1;
                            pos2(1), pos2(2)-1; pos2(1), pos2(2)+1]';
    else  % Horizontal movement
        adjacent_positions = [pos1(1)-1, pos1(2); pos1(1)+1, pos1(2);
                            pos2(1)-1, pos2(2); pos2(1)+1, pos2(2)]';
    end
end

function [path, found] = findWindowedPath(grid, start, goal, reservations, window)
    % A* search with windowed cooperative constraints
    openList = struct('pos', {}, 'g', {}, 'h', {}, 'f', {}, 'parent', {}, 'time', {});
    closedList = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Initialize start node
    start_node = struct('pos', start, 'g', 0, 'h', euclidean(start, goal), ...
        'f', 0, 'parent', [], 'time', 1);
    start_node.f = start_node.g + start_node.h;
    openList(1) = start_node;
    
    while ~isempty(openList)
        % Find node with minimum f value
        [~, curr_idx] = min([openList.f]);
        current = openList(curr_idx);
        openList(curr_idx) = [];
        
        % Check if goal reached
        if isequal(current.pos, goal)
            path = reconstructPath(current);
            found = true;
            return;
        end
        
        % Generate key for closed list
        key = sprintf('%d_%d_%d', current.pos(1), current.pos(2), current.time);
        
        % Skip if already explored
        if isKey(closedList, key)
            continue;
        end
        closedList(key) = current;
        
        % Check window limit
        if current.time > window
            continue;
        end
        
        % Generate neighbors
        moves = [-1 0; 1 0; 0 -1; 0 1; 0 0];  % Include wait action
        for i = 1:size(moves,1)
            new_pos = current.pos + moves(i,:);
            
            % Check grid bounds
            if any(new_pos < 1) || any(new_pos > size(grid))
                continue;
            end
            
            % Check for collisions
            if hasCollision(new_pos, current.time + 1, reservations)
                continue;
            end
            
            % Create neighbor node
            neighbor = struct('pos', new_pos, ...
                'g', current.g + 1, ...
                'h', euclidean(new_pos, goal), ...
                'parent', [], ...
                'time', current.time + 1);
            neighbor.f = neighbor.g + neighbor.h;
            neighbor.parent = current;
            
            % Add to open list
            openList(end+1) = neighbor;
        end
    end
    
    path = [];
    found = false;
end

function d = euclidean(a, b)
    % Euclidean distance heuristic (L2 norm)
    d = sqrt(sum((a - b).^2));
end

function collision = hasCollision(pos, time, reservations)
    collision = false;
    % Check exact position collisions
    for i = 1:length(reservations)
        if reservations(i).time == time && isequal(reservations(i).pos, pos)
            collision = true;
            return;
        end
    end
    
    % Check for crossing paths (robots swapping positions)
    if time > 1
        for i = 1:length(reservations)
            if reservations(i).time == time - 1 && isequal(reservations(i).pos, pos)
                for j = 1:length(reservations)
                    if reservations(j).time == time && ...
                       isequal(reservations(j).pos, pos) && ...
                       reservations(i).robot ~= reservations(j).robot
                        collision = true;
                        return;
                    end
                end
            end
        end
    end
end

function path = reconstructPath(node)
    % Reconstruct path from goal to start
    path = [];
    while ~isempty(node)
        path = [node.pos; path];
        node = node.parent;
    end
end

function temp_dest = findTemporaryDestination(grid, start, goal, reservations)
    % Find a temporary destination when direct path is blocked
    midpoint = round((start + goal) / 2);
    
    % Try positions around midpoint
    offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    for i = 1:size(offsets,1)
        candidate = midpoint + offsets(i,:);
        if all(candidate >= 1) && all(candidate <= size(grid)) && ...
           ~hasCollision(candidate, 1, reservations)
            temp_dest = candidate;
            return;
        end
    end
    
    % If no good position found, return midpoint
    temp_dest = midpoint;
end