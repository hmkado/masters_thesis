function [path, result] = findPath(grid, start, destination, rstart)
    % Adaptive path finding algorithm that handles different movement patterns
    % Input:
    %   grid: 2D matrix representing the map
    %   start: starting position [row, col]
    %   destination: destination position [row, col]
    %   rstart: movement pattern flag (1: bottom-to-top, 0: random-to-random)
    % Output:
    %   path: Array of positions representing the path from start to destination
    %   result: Status of path finding (0: optimal, 1: suboptimal, 2: emergency)
    
    if nargin < 4
        rstart = 0;  % Default to random-to-random pattern
    end
    
    if any(start < 1) || any(start > size(grid)) || any(destination < 1) || any(destination > size(grid))
        error('Start or destination position is outside the grid.');
    end
    
    % Define movement costs based on pattern
    if rstart == 1
        % Bottom-to-top pattern: Encourage upward movement, discourage downward
        moves = struct('coords', [-1, 0;  0, -1;  0, 1;  1, 0], ...  % up, left, right, down
                      'weights', [0.8;    1.0;    1.0;   1.5]);      % Lower cost for upward movement
    else
        % Random pattern: More balanced movement costs
        moves = struct('coords', [-1, 0;  1, 0;  0, -1;  0, 1], ...  % up, down, left, right
                      'weights', [1.0;    1.0;    1.0;   1.0]);      % Equal weights
    end
    
    % Define strategies based on movement pattern
    if rstart == 1
        strategies = {
            % Bottom-to-top strategies
            struct('obstacles', [1, 2, 3], 'zone_weights', createBottomTopZoneWeights(size(grid), destination)), 
            struct('obstacles', [1, 2],    'zone_weights', createBottomTopZoneWeights(size(grid), destination, 0.8)),
            struct('obstacles', [1],       'zone_weights', ones(size(grid)))  % Emergency
        };
    else
        strategies = {
            % Random movement strategies
            struct('obstacles', [1, 2, 3], 'zone_weights', createRandomZoneWeights(size(grid), start, destination)),
            struct('obstacles', [1, 2],    'zone_weights', createRandomZoneWeights(size(grid), start, destination, 0.8)),
            struct('obstacles', [1],       'zone_weights', ones(size(grid)))  % Emergency
        };
    end
    
    % Try strategies in order
    result = 0;
    for i = 1:length(strategies)
        try
            path = findPathWithStrategy(grid, start, destination, moves, strategies{i});
            result = i - 1;
            break;
        catch
            continue;
        end
    end
    
    if ~exist('path', 'var')
        error('No path found with any strategy.');
    end
end

function weights = createBottomTopZoneWeights(gridSize, destination, intensity)
    % Create weight matrix encouraging upward movement and proper spacing
    if nargin < 3
        intensity = 1.0;
    end
    
    weights = ones(gridSize);
    [rows, cols] = gridSize;
    
    % Create vertical gradient (higher costs near bottom)
    for i = 1:rows
        gradient_factor = 1 + (intensity * 0.2 * (i/rows));
        weights(i,:) = weights(i,:) * gradient_factor;
    end
    
    % Reduce weights near destination column to encourage proper spacing
    dest_col = destination(2);
    for j = 1:cols
        col_distance = abs(j - dest_col);
        if col_distance > 0
            weights(:,j) = weights(:,j) * (1 + 0.1 * col_distance * intensity);
        end
    end
    
    % Ensure destination has normal weight
    weights(destination(1), destination(2)) = 1.0;
end

function weights = createRandomZoneWeights(gridSize, start, destination, intensity)
    % Create weight matrix for random movement pattern
    if nargin < 4
        intensity = 1.0;
    end
    
    weights = ones(gridSize);
    [rows, cols] = gridSize;
    
    % Calculate direct path zone
    direct_path = bresenham(start(1), start(2), destination(1), destination(2));
    
    % Create weighted zones around direct path
    for i = 1:rows
        for j = 1:cols
            min_dist_to_path = inf;
            for k = 1:size(direct_path, 1)
                dist = sqrt((i-direct_path(k,1))^2 + (j-direct_path(k,2))^2);
                min_dist_to_path = min(min_dist_to_path, dist);
            end
            weights(i,j) = 1 + (0.15 * min_dist_to_path * intensity);
        end
    end
    
    % Ensure start and destination have normal weights
    weights(start(1), start(2)) = 1.0;
    weights(destination(1), destination(2)) = 1.0;
end

function path = findPathWithStrategy(grid, start, destination, moves, strategy)
    % A* implementation with strategic weighting
    openList = [];
    closedList = false(size(grid));
    
    % Combine base weights with zone weights and traffic density
    costGrid = strategy.zone_weights .* createTrafficDensityGrid(grid);
    
    % Initialize starting node
    startNode = struct('position', start, ...
                      'g', 0, ...
                      'h', heuristic(start, destination), ...
                      'parent', [], ...
                      'path_flexibility', calculateFlexibility(start, grid));
    startNode.f = startNode.g + startNode.h;
    openList = [openList; startNode];
    
    while ~isempty(openList)
        [~, idx] = min([openList.f]);
        currentNode = openList(idx);
        openList(idx) = [];
        
        if isequal(currentNode.position, destination)
            path = reconstructPath(currentNode);
            return;
        end
        
        closedList(currentNode.position(1), currentNode.position(2)) = true;
        
        for i = 1:size(moves.coords, 1)
            move = moves.coords(i, :);
            moveWeight = moves.weights(i);
            neighbor = currentNode.position + move;
            
            if isValidPosition(neighbor, size(grid)) && ...
               ~ismember(grid(neighbor(1), neighbor(2)), strategy.obstacles) && ...
               ~closedList(neighbor(1), neighbor(2))
                
                flexibility = calculateFlexibility(neighbor, grid);
                moveCost = moveWeight * costGrid(neighbor(1), neighbor(2)) * (1.2 - flexibility);
                tentativeG = currentNode.g + moveCost;
                
                neighborIdx = findPositionInList(openList, neighbor);
                
                if isempty(neighborIdx) || tentativeG < openList(neighborIdx).g
                    neighborNode = struct('position', neighbor, ...
                                        'g', tentativeG, ...
                                        'h', heuristic(neighbor, destination), ...
                                        'parent', currentNode, ...
                                        'path_flexibility', flexibility);
                    neighborNode.f = neighborNode.g + neighborNode.h;
                    
                    if isempty(neighborIdx)
                        openList = [openList; neighborNode];
                    else
                        openList(neighborIdx) = neighborNode;
                    end
                end
            end
        end
    end
    
    error('No path found.');
end

function density = createTrafficDensityGrid(grid)
    % Create traffic density cost grid
    density = ones(size(grid));
    [rows, cols] = size(grid);
    
    for i = 1:rows
        for j = 1:cols
            if grid(i,j) > 0
                % Add increasing cost around occupied cells
                radius = 2;
                for di = -radius:radius
                    for dj = -radius:radius
                        ni = i + di;
                        nj = j + dj;
                        if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols
                            distance = sqrt(di^2 + dj^2);
                            density(ni,nj) = density(ni,nj) + (1.3^(radius-distance));
                        end
                    end
                end
            end
        end
    end
end

function flexibility = calculateFlexibility(position, grid)
    % Calculate movement flexibility at position
    moves = [-1, 0; 1, 0; 0, -1; 0, 1];
    available = 0;
    potential = 0;
    
    for i = 1:size(moves, 1)
        neighbor = position + moves(i, :);
        if isValidPosition(neighbor, size(grid))
            potential = potential + 1;
            if grid(neighbor(1), neighbor(2)) == 0
                available = available + 1;
            end
        end
    end
    
    flexibility = available / potential;
end

% Helper functions remain the same
function h = heuristic(pos1, pos2)
    manhattan = sum(abs(pos1 - pos2));
    euclidean = norm(pos1 - pos2);
    h = 0.6 * manhattan + 0.4 * euclidean;
end

function valid = isValidPosition(pos, gridSize)
    valid = all(pos >= 1) && all(pos <= gridSize);
end

function path = reconstructPath(node)
    path = [];
    while ~isempty(node)
        path = [node.position; path];
        node = node.parent;
    end
end

function idx = findPositionInList(list, position)
    idx = find(arrayfun(@(x) isequal(x.position, position), list));
end

function coords = bresenham(x1, y1, x2, y2)
    % Bresenham's line algorithm for direct path calculation
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    steep = dy > dx;
    
    if steep
        [x1, y1] = swap(x1, y1);
        [x2, y2] = swap(x2, y2);
    end
    
    if x1 > x2
        [x1, x2] = swap(x1, x2);
        [y1, y2] = swap(y1, y2);
    end
    
    dx = x2 - x1;
    dy = abs(y2 - y1);
    error = dx / 2;
    ystep = (y1 < y2) * 2 - 1;
    
    y = y1;
    coords = zeros(dx + 1, 2);
    for x = x1:x2
        if steep
            coords(x-x1+1,:) = [y, x];
        else
            coords(x-x1+1,:) = [x, y];
        end
        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end
end

function [a, b] = swap(a, b)
    temp = a;
    a = b;
    b = temp;
end