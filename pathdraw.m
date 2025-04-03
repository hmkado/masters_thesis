clear all;
close all;
clc;

% control
testimg = 0;
continuous = 0;
rstart = 0;
testbots = [1 2 3 4 5 7 8];
trial_no = 11;
time_limit = 120;
webcam_num = 2;
maxcount = 4;

% setting
markerSizeInMM = 70;
markerFamily = "DICT_4X4_250";
ncolrow = [6 6];

locrec = [];
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

data = load("camp12.mat");
intrinsics = data.cameraParams.Intrinsics;

img = imread(sprintf("trial_%d\\base.png",trial_no));

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
se = strel('disk', 4);
bw = imclose(bw, se);
skel = bwskel(bw,'MinBranchLength',50);
branch_points = bwmorph(skel, 'branchpoints');
[y, x] = find(branch_points);
intersections = [x, y];
intersections = uniquetol(intersections, 5, 'ByRows', true, 'DataScale', 1);

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

% figure(1);
% image(warpedimg);colormap("gray");
% hold on;
% scatter(sortedPoints(:,1), sortedPoints(:,2),'r*');

path = cell(8,1);
for frame = 2:14
    img = imread(sprintf("trial_%d\\si_%d.png",trial_no,frame));
    [~,camIntrinsics] = undistortImage(img,intrinsics);
    [ids,locs,poses] = readArucoMarker(img,markerFamily,camIntrinsics,markerSizeInMM);
    
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
    
    rob_locs = zeros(8,2);
    rloc{frame-1} = zeros(6);
    for n = testbots
        k = dsearchn(sortedPoints,warpedRobotCoords(n,:));
        [h c] = ind2sub(ncolrow,k);
        rob_locs(n,:) = [c h];
        path{n} = [path{n}; c h];
        rloc{frame-1}(c,h) = n;
    end
    
end

b = [0 20 88 115 147 186 219 260 287 324 378];

figure(2);
colormap_turbo = turbo;
new_colormap = [1 1 1; colormap_turbo];
for n = 1:13
    data = rloc{n};
    heatmap(data, 'Colormap', new_colormap);
    pause(1);
end

%%
figure(2);
image(warpedgray);colormap("gray");
hold on;
scatter(sortedPoints(:,1), sortedPoints(:,2),'r*');
shift = [-.4 -.3 -.2 -.1 0 .1 .2 .3 .4].*30;
for n = testbots
    clear truepathx
    clear truepathy
    for m = 1:size(path{n},1)
        truepathx(m) = gridcoord{path{n}(m,1),path{n}(m,2)}(1) + shift(n);
        truepathy(m) = gridcoord{path{n}(m,1),path{n}(m,2)}(2) + shift(n);
    end
    p(n) = plot(truepathx,truepathy,"LineWidth",4,"LineStyle","-.");
end

hold off;