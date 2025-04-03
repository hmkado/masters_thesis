function [path, result] = findPath(grid, start, destination)
    % Input:
    %   grid: 2D matrix representing the map
    %   start: starting position [row, col]
    %   destination: destination position [row, col]
    % Output:
    %   path: Array of positions representing the path from start to destination
    
    % Check if start and destination are within the grid
    if any(start < 1) || any(start > size(grid)) || any(destination < 1) || any(destination > size(grid))
        error('Start or destination position is outside the grid.');
    end
    
    % Define possible movements (up, down, left, right)
    moves = [-1, 0; 1, 0; 0, -1; 0, 1];
    num_directions = size(moves, 1);
    random_indices = randperm(num_directions);
    randomized_directions = moves(random_indices, :);
    
    obs = {[1, 2, 3], [1, 2], [1]};
    result = 1;
    % Attempt to find a path avoiding both 1s and 2s
    for n = 1:3
        try
            path = findPathHelper(grid, start, destination, randomized_directions, obs{n});
            if n == 1
                result = 0;
            end
            break;
        end
    end

    % try
    %     path = findPathHelper(grid, start, destination, randomized_directions, [1, 2]);
    %     result = 0;
    % catch
    %     % If unsuccessful, try to find a path avoiding only 1s
    %     path = findPathHelper(grid, start, destination, randomized_directions, 1);
    %     result = 1;
    % end
end

function path = findPathHelper(grid, start, destination, moves, obstacles)
    % A helper function for finding a path while avoiding specified obstacles
    
    % Initialize open and closed lists
    openList = [];
    closedList = false(size(grid));
    
    % Initialize starting node
    startNode.g = 0;
    startNode.h = heuristic(start, destination);
    startNode.f = startNode.g + startNode.h;
    startNode.position = start;
    startNode.parent = [];
    
    % Add starting node to open list
    openList = [openList; startNode];
    
    while ~isempty(openList)
        % Find node with the minimum f value in the open list
        [~, idx] = min([openList.f]);
        currentNode = openList(idx);
        
        % Remove current node from open list
        openList(idx) = [];
        
        % Mark current node as visited
        closedList(currentNode.position(1), currentNode.position(2)) = true;
        
        % Check if the destination is reached
        if isequal(currentNode.position, destination)
            % Reconstruct the path
            path = reconstructPath(currentNode);
            return;
        end
        
        % Generate neighboring nodes
        for move = moves'
            neighbor = currentNode.position + move';
            
            % Check if the neighbor is within the grid
            if all(neighbor >= 1) && all(neighbor <= size(grid))
                % Check if the neighbor is traversable (not an obstacle)
                if ~ismember(grid(neighbor(1), neighbor(2)), obstacles)
                    % Check if neighbor is not in the closed list
                    if ~closedList(neighbor(1), neighbor(2))
                        % Calculate tentative g value
                        tentativeG = currentNode.g + 1; % Assuming uniform cost for each step
                        
                        % Check if neighbor is not in the open list or has a lower g value
                        neighborIdx = findPositionInList(openList, neighbor);
                        if isempty(neighborIdx) || tentativeG < openList(neighborIdx).g
                            % Add or update the neighbor in the open list
                            if isempty(neighborIdx)
                                neighborNode.position = neighbor;
                                neighborNode.parent = currentNode;
                                neighborNode.g = tentativeG;
                                neighborNode.h = heuristic(neighbor, destination);
                                neighborNode.f = neighborNode.g + neighborNode.h;
                                openList = [openList; neighborNode];
                            else
                                openList(neighborIdx).parent = currentNode;
                                openList(neighborIdx).g = tentativeG;
                                openList(neighborIdx).f = openList(neighborIdx).g + openList(neighborIdx).h;
                            end
                        end
                    end
                end
            end
        end
    end
    
    % No path found
    error('No path found.');
end


function h = heuristic(pos1, pos2)
    % Manhattan distance heuristic (L1 norm)
    h = sum(abs(pos1 - pos2));
end

function path = reconstructPath(node)
    % Reconstruct the path from the destination to the start
    path = [];
    while ~isempty(node)
        path = [node.position; path];
        node = node.parent;
    end
end

function idx = findPositionInList(list, position)
    % Find the index of a position in the list
    idx = find(arrayfun(@(x) isequal(x.position, position), list));
end
