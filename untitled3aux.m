rgraph = rgraph.*0;
online = zeros(numbots,1);
online(testbots) = 1;
finished = zeros(numbots,1);
loopcount = 0;

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
mh = [1 1;1 ncolrow(2);ncolrow(1) 1;ncolrow(1) ncolrow(2)];
for n = testbots
    k = dsearchn(sortedPoints,warpedRobotCoords(n,:));
    [h c] = ind2sub(ncolrow,k);
    agv(n).grid = [c h];
    agv(n).nextgrid = agv(n).grid;
    rgraph(c,h) = rgraph(c,h) + n;

    % if all([c h] == agv(n).dest)
        agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
        dest(n,:) = agv(n).dest;
        [dup,~] = findrows(dest);
        while all(agv(n).grid == agv(n).dest) || dup || ismember(agv(n).dest,mh,'row')
            dest(n,:) = agv(n).dest;
            if rstart == 0
                agv(n).dest = [randi([1 ncolrow(2)]) randi([1 ncolrow(1)])];
            else
                agv(n).dest = [1 randi([1 ncolrow(2)])];
            end
            [dup,~] = findrows(dest);
        end
    % end

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

    clear truepathx
    clear truepathy
end
istep = istep + 1;
imwrite(warpedgray,sprintf("trial_%d/si_%d.png",trial_no,istep));

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