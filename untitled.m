clear all;
close all;
clc;
cd C:\Users\JR\Documents\two

% control
testimg = 0;
continuous = 0;
rstart = 1;
testbots = [1 2 3 4 5 6];
trial_no = 9;
time_limit = 120;
webcam_num = 2;
maxcount = 4;

% setting
markerSizeInMM = 70;
markerFamily = "DICT_4X4_250";
ncolrow = [6 6];

grid = zeros(ncolrow(2),ncolrow(1));

rgraph = zeros(ncolrow(2),ncolrow(1));
numbots = 12;
robotCoords = zeros(numbots,2);
robatt = zeros(numbots,1);
rotationAngles = zeros(numbots,1);
robheading = cell(numbots,1);
bottotal = size(testbots,2);
rr = randperm(numbots-4);
intersections = zeros(1);

% if testimg == 0
    data = load("camp12.mat");
% else
%     data = load("cameraParams.mat");
% end
intrinsics = data.cameraParams.Intrinsics;

% wifi setting
esp8266_ip = {'192.168.0.10';'192.168.0.6';'192.168.0.5';'192.168.0.8';'192.168.0.12';'192.168.0.14';'192.168.0.11';'192.168.0.13'};
port = 80;

% cam setup
if testimg == 0
    clear('cam');
    cam = webcam(webcam_num);
    % cam.Resolution = cam.AvailableResolutions{1};
end

% Calibrate corner refs
while true
while true
    if testimg == 0
        img = snapshot(cam);
    else
        img = imread("test.png");
    end
    gimg = rgb2gray(img);
    [ids,locs,detectedFamily] = readArucoMarker(gimg,markerFamily);

    % center of marker
    numMarkers = length(ids);
    for n = 1:numMarkers
        loc = locs(:,:,n);
        center(n,:) = mean(loc);
        
        markerRadius = 6;
        numCorners = size(loc,1);
        markerPosition = [center(n,:),markerRadius];
        % img = insertShape(img,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
        % img = insertText(img,center(n,:),ids(n));
    end
    length(ids)

    if length(ids) >= 4 & all(ismember(1:4,ids))
        break;
    end
end

% image adjust
imgWidth = 1000;  % You can adjust this value
imgHeight = round(imgWidth * (1080/1920));
destCorners = [
    0 0;                % Top-left
    imgWidth 0;         % Top-right
    0 imgHeight;        % Bottom-left
    imgWidth imgHeight  % Bottom-right
];

% warp with corner ref
[~, idx] = ismember(1:4, ids);
ref_pts = center(idx, :);

tform = fitgeotrans(ref_pts, destCorners, 'projective');
outputView = imref2d([imgHeight imgWidth]);
warpedimg = imwarp(gimg, tform, 'OutputView', outputView);
warpedimg = (warpedimg);

% extract vertices
threshold = graythresh(warpedimg);
bw = imbinarize(warpedimg, threshold);

bw = ~bw;
bw = bwareaopen(bw, 100);
se = strel('disk', 3);
bw = imclose(bw, se);
skel = bwskel(bw,'MinBranchLength',50);
branch_points = bwmorph(skel, 'branchpoints');
[y, x] = find(branch_points);
intersections = [x, y];
intersections = uniquetol(intersections, 5, 'ByRows', true, 'DataScale', 1);

if size(intersections,1) == prod(ncolrow)
    break;
end
end


% lbcorner = [min(intersections(:,1)) max(intersections(:,2))];
% ltcorner = [min(intersections(:,1)) min(intersections(:,2))];
% rbcorner = [max(intersections(:,1)) max(intersections(:,2))];
% rtcorner = [max(intersections(:,1)) min(intersections(:,2))];
% intersections = [intersections;lbcorner;ltcorner;rbcorner;rtcorner];

% sort grid
[~, sortIdxY] = sort(intersections(:,2));
pointsSortedY = intersections(sortIdxY,:);
sortedPoints = zeros(size(intersections));
for row = 1:ncolrow(2)
    rowStart = (row-1)*ncolrow(1) + 1;
    rowEnd = row*ncolrow(1);
    rowPoints = pointsSortedY(rowStart:rowEnd, :);
    [~, sortIdxX] = sort(rowPoints(:,1));
    sortedPoints(rowStart:rowEnd, :) = rowPoints(sortIdxX, :);
end

k = 1;
for n = 1:ncolrow(2)
    for m = 1:ncolrow(1)
        gridcoord{n,m} = sortedPoints(k,:);
        k = k + 1;
    end
end

figure(1);
image(warpedimg);colormap("gray");
hold on;
scatter(sortedPoints(:,1), sortedPoints(:,2),'r*');

%% Robot prep
% setup
rgraph = rgraph.*0;
online = zeros(numbots,1);
online(testbots) = 1;
finished = zeros(numbots,1);
loopcount = 0;
dest = zeros(numbots,2);

while true
    if testimg == 0
        clear('cam');
        cam = webcam(webcam_num);
        % cam.Resolution = cam.AvailableResolutions{1};
        img = snapshot(cam);
    else
        img = imread("test_marker.png");
    end
    
    [~,camIntrinsics] = undistortImage(img,intrinsics);
    [ids,locs,poses] = readArucoMarker(img,markerFamily,camIntrinsics,markerSizeInMM);
    ids
    if all(ismember([testbots+4],ids))
        break;
    end
end

% ArUco pose
worldPoints = [0 0 0; markerSizeInMM/2 0 0; 0 markerSizeInMM/2 0; 0 0 markerSizeInMM/2];
for i = 1:length(poses)
    if ids(i) > 4
        imagePoints = world2img(worldPoints,poses(i),camIntrinsics);
        
        axesPoints = [imagePoints(1,:) imagePoints(2,:);
                    imagePoints(1,:) imagePoints(3,:);
                        imagePoints(1,:) imagePoints(4,:)];
        % img = insertShape(img, "Line", axesPoints, ...
        % Color = ["red","green","blue"], Line 
        % Width=10);
        
        R = poses(i).Rotation;
        
        % Calculate rotation angle around z-axis (in radians)
        theta = atan2(R(2,1), R(1,1));
        
        % Convert to degrees
        rotationAngles(ids(i)) = rad2deg(theta);
        rotationAngles(rotationAngles > 180) = rotationAngles(rotationAngles > 180) - 360;
        rotationAngles(rotationAngles < -180) = rotationAngles(rotationAngles < -180) + 360;
        if rotationAngles(ids(i)) >= -45 && rotationAngles(ids(i)) <= 45
            robheading{ids(i)} = 'down';
        elseif rotationAngles(ids(i)) > 45 && rotationAngles(ids(i)) < 135
            robheading{ids(i)} = 'right';
        elseif rotationAngles(ids(i)) < -45 && rotationAngles(ids(i)) > -135
            robheading{ids(i)} = 'left';
        else
            robheading{ids(i)} = 'up';
        end
    end
end

% center of marker
numMarkers = length(ids);
for n = 1:length(ids)
    loc = locs(:,:,n);
    center(ids(n),:) = mean(loc);
end

% adjust marker coords
idx = find(ids>4);
robotCoords = center(5:end, :);
robatt(ids(idx)) = 1;
[warpedRobotX, warpedRobotY] = transformPointsForward(tform, robotCoords(:,1), robotCoords(:,2));
warpedRobotCoords = [warpedRobotX, warpedRobotY];

% image adjust
imgWidth = 1000;  % You can adjust this value
imgHeight = round(imgWidth * (1080/1920));
destCorners = [
    0 0;                % Top-left
    imgWidth 0;         % Top-right
    0 imgHeight;        % Bottom-left
    imgWidth imgHeight  % Bottom-right
];
tform = fitgeotrans(ref_pts, destCorners, 'projective');
outputView = imref2d([imgHeight imgWidth]);
warpedimg = imwarp(img, tform, 'OutputView', outputView);
warpedgray = (rgb2gray(warpedimg));

figure(3);
image(warpedgray);colormap("gray");
hold on;
scatter(sortedPoints(:,1), sortedPoints(:,2),'r*');
% text(warpedRobotCoords(:,1),warpedRobotCoords(:,2),{'1','2'}); % ,'3','4'

% robot setup
for n = testbots
    k = dsearchn(sortedPoints,warpedRobotCoords(n,:));
    [h c] = ind2sub(ncolrow,k);
    agv(n).grid = [c h];
    agv(n).nextgrid = agv(n).grid;
    rgraph(c,h) = rgraph(c,h) + n;

    if rstart == 0
        agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
    else
        agv(n).dest = [1 randi([1 ncolrow(2)])];
    end
    dest(n,:) = agv(n).dest;
    [dup,~] = findrows(dest);
    while all(agv(n).grid == agv(n).dest) || dup
        dest(n,:) = agv(n).dest;
        if rstart == 0
            agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
        else
            agv(n).dest = [1 randi([1 ncolrow(2)])];
        end
        [dup,~] = findrows(dest);
    end

    agv(n).ip = esp8266_ip{n};
    agv(n).step = 1;
    agv(n).pstep = agv(n).step;
    agv(n).evade = 0; %0 off 1 on 2 threshold met
    agv(n).wait = 0;
    agv(n).threshold = rr(n);
    agv(n).heading = robheading{4+n};
    agv(n).canmove = 0;
    agv(n).count = 0;
    
    % tcpClient(n) = tcpclient(esp8266_ip(n,:), port);

    [tpath{n}, ~] = findPath(grid,agv(n).grid,agv(n).dest);
    [agv(n).moves, turns] = analyzePath(tpath{n}, robheading{4+n});

    for m = 1:size(tpath{n},1)
        truepathx(m) = gridcoord{tpath{n}(m,1),tpath{n}(m,2)}(1);
        truepathy(m) = gridcoord{tpath{n}(m,1),tpath{n}(m,2)}(2);
    end
    p(n) = plot(truepathx,truepathy,"LineWidth",2,"LineStyle","-.");

    [subjective_moves, turns] = analyzePath(tpath{n}, agv(n).heading);

    [nextmove, turns] = analyzePath([tpath{n}(agv(n).step,:); tpath{n}(agv(n).step+1,:)], agv(n).heading);

    sprintf("%d : Moves ", n)
    subjective_moves

    clear truepathx
    clear truepathy
end
saveas(figure(3),sprintf("trial_%d/initial.png",trial_no));

%%
%run
MAX_WAIT_TIME = 100000; % Maximum wait time in seconds
lastMoveTime = cell(1, length(testbots));
status = zeros(1,size(testbots,2));

% data collection
collisionct = [];
completionct = [];
objective_step = 1;

if testimg == 0
    for n = testbots
        tcpClient{n} = tcpclient(agv(n).ip, port);
    end
    clear('cam');
    cam = webcam(webcam_num);
    % cam.Resolution = cam.AvailableResolutions{1};
end
start_time = tic;
while true
    %---checking completion---
    while ~all(online(testbots) == 1)
        for n = testbots
            if ~online(n)
                if tcpClient{n}.BytesAvailable > 0
                    try
                        data = read(tcpClient{n}, tcpClient{n}.BytesAvailable, 'uint8');
                        if ~isempty(data) && data(end) == 49  % Check for ASCII '1'
                            online(n) = 1;
                            lastMoveTime{n} = tic;
                            disp(['Robot ', num2str(n), ' completed move. Received: ', num2str(data)]);
                        else
                            disp(['Robot ', num2str(n), ' sent unexpected data: ', num2str(data)]);
                        end
                        pause(0.05);  % Short pause before flushing
                        flush(tcpClient{n}); % Clear any remaining data
                    catch ME
                        disp(['Error reading from robot ', num2str(n), ': ', ME.message]);
                        % Optionally, you might want to set online(n) = 1 here to avoid getting stuck
                    end
                elseif agv(n).canmove == 0
                    online(n) = 1;
                elseif toc(lastMoveTime{n}) > MAX_WAIT_TIME
                    disp(['Robot ', num2str(n), ' timed out. Assuming completion.']);
                    online(n) = 1;
                end
            end
        end
        disp(['Current online status: ', mat2str(online(testbots))]);
        pause(0.1); % Short pause to prevent CPU overuse
    end
    %-------------------------

    %---take picture and read markers
    online = online.*0;
    maxAttempts = 10;
    while true
    % for attempt = 1:maxAttempts
        % try
            clear('cam');
            clear ids;
            clear img;
            clear locs;
            clear poses;
            cam = webcam(webcam_num);
            % cam.Resolution = cam.AvailableResolutions{1};
            % pause(0.1); % Give the camera some time to initialize
    
            img = snapshot(cam);
            
            [~,camIntrinsics] = undistortImage(img,intrinsics);
            [ids,locs,poses] = readArucoMarker(img,markerFamily,camIntrinsics,markerSizeInMM);
            
            if length(ids) >= 4+size(testbots,2) && all(ismember([1:4,testbots+4],ids))
                break;
            else
                % disp(['Attempt ', num2str(attempt), ': Not all markers detected. Retrying...']);
            end
        % catch ME
        %     disp(['Error in image capture/processing (attempt ', num2str(attempt), '): ', ME.message]);
        %     if attempt == maxAttempts
        %         error('Maximum attempts reached. Unable to capture and process image.');
        %     end
        % end
        % pause(.1); % Wait a bit before next attempt
    end

    % ArUco pose
    worldPoints = [0 0 0; markerSizeInMM/2 0 0; 0 markerSizeInMM/2 0; 0 0 markerSizeInMM/2];
    for i = 1:length(poses)
        if ids(i) > 4
            imagePoints = world2img(worldPoints,poses(i),camIntrinsics);
            
            axesPoints = [imagePoints(1,:) imagePoints(2,:);
                        imagePoints(1,:) imagePoints(3,:);
                        imagePoints(1,:) imagePoints(4,:)];
            % img = insertShape(img, "Line", axesPoints, ...
            % Color = ["red","green","blue"], Line 
            % Width=10);
            
            R = poses(i).Rotation;
            
            % Calculate rotation angle around z-axis (in radians)
            theta = atan2(R(2,1), R(1,1));
            
            % Convert to degrees
            rotationAngles(ids(i)) = rad2deg(theta);
            rotationAngles(rotationAngles > 180) = rotationAngles(rotationAngles > 180) - 360;
            rotationAngles(rotationAngles < -180) = rotationAngles(rotationAngles < -180) + 360;
            if rotationAngles(ids(i)) >= -45 && rotationAngles(ids(i)) <= 45
                robheading{ids(i)} = 'down';
            elseif rotationAngles(ids(i)) > 45 && rotationAngles(ids(i)) < 135
                robheading{ids(i)} = 'right';
            elseif rotationAngles(ids(i)) < -45 && rotationAngles(ids(i)) > -135
                robheading{ids(i)} = 'left';
            else
                robheading{ids(i)} = 'up';
            end
        end
    end
    
    % center of marker
    numMarkers = length(ids);
    for m = 1:length(ids)
        loc = locs(:,:,m);
        center(ids(m),:) = mean(loc);
    end

    % adjust marker coords
    idx = find(ids>4);
    robotCoords = center(5:end, :);
    robatt(ids(idx)) = 1;
    [warpedRobotX, warpedRobotY] = transformPointsForward(tform, robotCoords(:,1), robotCoords(:,2));
    warpedRobotCoords = [warpedRobotX, warpedRobotY];

    % image adjust
    imgWidth = 1000;  % You can adjust this value
    imgHeight = round(imgWidth * (1080/1920));
    destCorners = [
        0 0;                % Top-left
        imgWidth 0;         % Top-right
        0 imgHeight;        % Bottom-left
        imgWidth imgHeight  % Bottom-right
    ];
    tform = fitgeotrans(ref_pts, destCorners, 'projective');
    outputView = imref2d([imgHeight imgWidth]);
    warpedimg = imwarp(img, tform, 'OutputView', outputView);
    warpedgray = (rgb2gray(warpedimg));

    figure(3);
    image(warpedgray);colormap("gray");
    hold on;
    scatter(sortedPoints(:,1), sortedPoints(:,2),'r*');

    for n = testbots
        for m = 1:size(tpath{n},1)
            truepathx(m) = gridcoord{tpath{n}(m,1),tpath{n}(m,2)}(1);
            truepathy(m) = gridcoord{tpath{n}(m,1),tpath{n}(m,2)}(2);
        end
        set(p(n),'XData',truepathx,'YData',truepathy)
        clear truepathx;
        clear truepathy;
    end

    saveas(figure(2),sprintf("trial_%d/frame_%d.png",trial_no,objective_step));
    %-------------------------

    % mark robot location
    rgraph = rgraph.*0;
    for n = testbots
        k = dsearchn(sortedPoints,warpedRobotCoords(n,:));
        [h c] = ind2sub(ncolrow,k);
        agv(n).grid = [c h];
        agv(n).nextgrid = [0 0];
        agv(n).canmove = 0;
        rgraph(c,h) = rgraph(c,h) + n;
        disp(['agv loc vs path loc: ', mat2str([n agv(n).grid tpath{n}(agv(n).step,:)])]);
        % if agv(n).step < size(tpath{n},1)
        %     rgraph(tpath{n}(agv(n).step+1,1),tpath{n}(agv(n).step+1,2)) = rgraph(tpath{n}(agv(n).step+1,1),tpath{n}(agv(n).step+1,2)) + n;
        % end
        agv(n).heading = robheading{4+n};
        status(n) = agv(n).step;
    end
    disp(['agv steps: ', mat2str(status)]);

    %---calculate path/moves
    for n = testbots
        if ~finished(n)
            if all(agv(n).grid == tpath{n}(agv(n).step,:))
            else
                agv(n).step = 1;
                if rstart == 0
                    agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
                else
                    agv(n).dest = [randi([1 ncolrow(2)]) 1];
                end
                dest(n,:) = agv(n).dest;
                [dup,~] = findrows(dest);
                while all(agv(n).grid == agv(n).dest) || dup
                    dest(n,:) = agv(n).dest;
                    if rstart == 0
                        agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
                    else
                        agv(n).dest = [randi([1 ncolrow(2)]) 1];
                    end
                    [dup,~] = findrows(dest);
                end
                [tpath{n}, ~] = findPath(grid, agv(n).grid, agv(n).dest);
                continue;
            end

            % Check if robot has reached its destination
            if all(agv(n).grid == agv(n).dest) || agv(n).count > maxcount
                completionct = [completionct; n toc(start_time) objective_step];
                if continuous == 0
                    finished(n) = 1;
                else
                    % Assign new destination
                    agv(n).step = 1;
                    if rstart == 0
                        agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
                    else
                        agv(n).dest = [randi([1 ncolrow(2)]) 1];
                    end
                    dest(n,:) = agv(n).dest;
                    [dup,~] = findrows(dest);
                    while all(agv(n).grid == agv(n).dest) || dup
                        dest(n,:) = agv(n).dest;
                        if rstart == 0
                            agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
                        else
                            agv(n).dest = [randi([1 ncolrow(2)]) 1];
                        end
                        [dup,~] = findrows(dest);
                    end
                    [tpath{n}, ~] = findPath(grid, agv(n).grid, agv(n).dest);
                end
                agv(n).canmove = 0;
                continue;  % Skip to next robot
            end

            if agv(n).step < size(tpath{n}, 1)
                nextMove = tpath{n}(agv(n).step + 1, :);
                
                % Check for head-on collision
                isHeadOn = false;
                otherRobot = 0;
                for other = testbots
                    if other ~= n && agv(other).step < size(tpath{other}, 1)
                        otherNextMove = tpath{other}(agv(other).step + 1, :);
                        if all(nextMove == agv(other).grid) && all(otherNextMove == agv(n).grid)
                            isHeadOn = true;
                            otherRobot = other;
                            break;
                        end
                    end
                end
        
                if isHeadOn
                    collisionct = [collisionct; n toc(start_time) objective_step];
                    % Handle head-on collision
                    robot1ToDestination = all(nextMove == agv(n).dest);
                    robot2ToDestination = all(agv(n).grid == agv(otherRobot).dest);
    
                    if robot1ToDestination && robot2ToDestination
                        % Special case: robots are swapping positions to reach their destinations
                        % Try to move one robot to a temporary position
                        tempPosition = [];
                        for dir = [-1 0; 1 0; 0 -1; 0 1]'
                            newPos = agv(n).grid + dir;
                            if all(newPos >= 1) && all(newPos <= [ncolrow(2), ncolrow(1)]) && rgraph(newPos(1), newPos(2)) == 0
                                tempPosition = newPos;
                                break;
                            end
                        end
                        if ~isempty(tempPosition)
                            % Move robot n to temp position, then to final destination
                            tempGrid = grid;
                            for m = testbots
                                if m ~= n
                                    tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                                end
                            end
                            [newPath, success] = findPath(tempGrid, agv(n).grid, tempPosition);
                            if success
                                tpath{n} = [newPath; agv(n).dest];
                                agv(n).step = 1;
                                
                            else
                                agv(n).canmove = 0;
                            end
                        else
                            % If no temp position found, both robots wait
                            agv(n).canmove = 0;
                            agv(otherRobot).canmove = 0;
                        end
                    elseif robot1ToDestination
                        % Robot n has priority, other robot finds new path
                        tempGrid = grid;
                        for m = testbots
                            if m ~= otherRobot
                                tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                            end
                        end
                        [newPath, success] = findPath(tempGrid, agv(otherRobot).grid, agv(otherRobot).dest);
                        if success
                            tpath{otherRobot} = newPath;
                            agv(otherRobot).step = 1;
                        else
                            agv(otherRobot).canmove = 0;
                        end
                    elseif robot2ToDestination
                        % Other robot has priority, robot n finds new path
                        tempGrid = grid;
                        for m = testbots
                            if m ~= n
                                tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                            end
                        end
                        [newPath, success] = findPath(tempGrid, agv(n).grid, agv(n).dest);
                        if success
                            tpath{n} = newPath;
                            agv(n).step = 1;
                        else
                            agv(n).canmove = 0;
                        end
                    else
                        % Neither robot is at its destination, lower number has priority
                        if n < otherRobot
                            tempGrid = grid;
                            for m = testbots
                                if m ~= otherRobot
                                    tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                                end
                            end
                            [newPath, success] = findPath(tempGrid, agv(otherRobot).grid, agv(otherRobot).dest);
                            if success
                                tpath{otherRobot} = newPath;
                                agv(otherRobot).step = 1;
                            else
                                agv(otherRobot).canmove = 0;
                            end
                        else
                            tempGrid = grid;
                            for m = testbots
                                if m ~= n
                                    tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                                end
                            end
                            [newPath, success] = findPath(tempGrid, agv(n).grid, agv(n).dest);
                            if success
                                tpath{n} = newPath;
                                agv(n).step = 1;
                            else
                                agv(n).canmove = 0;
                            end
                        end
                    end
                elseif rgraph(nextMove(1), nextMove(2)) ~= 0
                    % Next move is occupied
                    collisionct = [collisionct; n toc(start_time) objective_step];
                    agv(n).canmove = 0;
                else
                    % Next move is free
                    rgraph(nextMove(1), nextMove(2)) = rgraph(nextMove(1), nextMove(2)) + n;
                    agv(n).canmove = 1;
                    agv(n).nextgrid = nextMove;
                    agv(n).step = agv(n).step + 1;
                end
            else
                % End of path reached, recalculate
                tempGrid = grid;
                for m = testbots
                    if m ~= n
                        tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
                    end
                end
                [newPath, success] = findPath(tempGrid, agv(n).grid, agv(n).dest);
                if success
                    tpath{n} = newPath;
                    agv(n).step = 1;
                    collisionct = [collisionct; n toc(start_time) objective_step];
                else
                    agv(n).canmove = 0;
                end
            end
        end
    end


    % send command
    objective_step = objective_step + 1;
    for n = testbots
        if agv(n).canmove == 1
            [action{n}, ~] = analyzePath([agv(n).grid; agv(n).nextgrid], agv(n).heading);
            flush(tcpClient{n}); % Flush before sending
            write(tcpClient{n}, int2str(action{n}{1}), 'uint8');
            pause(0.1); % Allow time for command to be sent
            flush(tcpClient{n}); % Flush after sending
            online(n) = 0; % Reset online status
            lastMoveTime{n} = tic; % Reset move timer
            agv(n).count = 0;
        else
            agv(n).count = agv(n).count + 1;
        end
    end

    if all(finished(testbots)) || toc(start_time) > time_limit
        break;
    end
end

function adjZeros = findfreecells(array, row, col, num)
    % Get the size of the array
    [n, m] = size(array);
    
    % Initialize the list of adjacent zeros
    adjZeros = [];

    % Check the four neighboring cells
    directions = [-1, 0; 1, 0; 0, -1; 0, 1];
    for i = 1:size(directions, 1)
        newRow = row + directions(i, 1);
        newCol = col + directions(i, 2);

        % Check if the new indices are within bounds
        if newRow >= 1 && newRow <= n && newCol >= 1 && newCol <= m
            % Check if the current cell contains zero
            if array(newRow, newCol) == 0 || (array(newRow, newCol) == num+1)
                % Add the indices to the list of adjacent zeros
                adjZeros = [adjZeros; newRow, newCol];
            end
        end
    end
end

function recalculatePathForRobot(robotNum)
    tempGrid = grid;
    for m = testbots
        if m ~= robotNum
            tempGrid(agv(m).grid(1), agv(m).grid(2)) = 1;
        end
    end
    [newPath, success] = findPath(tempGrid, agv(robotNum).grid, agv(robotNum).dest);
    if success
        tpath{robotNum} = newPath;
        agv(robotNum).step = 1;
        agv(robotNum).canmove = 1;
        collisionct = [collisionct; robotNum toc(start_time) objective_step];
    else
        agv(robotNum).canmove = 0;
    end
end

function valid = isValidPosition(pos, ncolrow)
    valid = all(pos >= 1) && all(pos <= [ncolrow(1), ncolrow(2)]);
end


function [dup, didx] = findrows(array)
    % Remove all-zero rows
    nonZeroRows = array(any(array, 2), :);
    
    % Find unique rows among non-zero rows
    [uniqueRows, ~, ic] = unique(nonZeroRows, 'rows');
    
    % Check if there are duplicates
    dup = size(uniqueRows, 1) < size(nonZeroRows, 1);
    
    if dup
        % Find which unique rows appear more than once
        duplicateIndices = find(histc(ic, 1:max(ic)) > 1);
        didx = uniqueRows(duplicateIndices, :);
    else
        didx = [];
    end
end

