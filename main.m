% function[collisionct, evasionct, station_in, station_out, heatmap,congestionmap,toshelf_cell,totarget_cell,tostation_cell,original_toshelf_cell,original_totarget_cell,original_tostation_cell, inrate, outrate, inot, outot, picktime, totalpath, activity, activation_time, conmap_area, avoidedpath, deposit, shelfinfo1,shelfinfo2, construction,item_in] = main(vis, num,item_scatter, pathrecon, demand, item_width, dist_dist, qsize, fill_limit, precis, transition)

% ---------------

clear all;
close all;
clc;

cd C:\Users\JR\Documents\two

pathrecon = 3;
item_scatter = 1; % starting condition: 0 empty % 1 rand filled
dist_dist = 3; % placement distance: 0 rand % 1 demand-based dist % 2 normal dist % 3 adaptive normal
num = 50;
vis = 1;
demand = 0; % item distribution: 0 uniform % 1 pareto
qsize = 50; % queue size
fill_limit = 320; % fill limit
precis = 1;
transition = 0; % distribution transition: 0 off

% ---------------

lmax = 26;
speed = 1;
time_limit = 40000;

graphsz = [29 35];
r = 4;
cellsz = 10;
margin = 20;
t = 0:0.1:2*pi+0.2;
turnspeed = 2*speed;
collmax = 50;
coloredstation = 2;
ccount = zeros(4,1);
rr = randperm(collmax, num);
fieldgraph = zeros(graphsz);
item_width = 4;
alpha = 1;         % pareto shape parameter
xm = 1;            % pareto scale parameter

C = orderedcolors("gem12");

% DC
collisionct = zeros(num,1);
evasionct = zeros(num,1);
station_in = zeros(4, 1);
station_out = zeros(4,1);
item_in = zeros(4, 1);
heatmap = zeros(graphsz);
congestionmap = zeros(graphsz);
toshelf_cell = cell(num, 1);
totarget_cell = cell(num, 1);
tostation_cell = cell(num, 1);
original_toshelf_cell = cell(num, 1);
original_totarget_cell = cell(num, 1);
original_tostation_cell = cell(num, 1);
picktime = cell(num, 1);
totalpath = cell(num, 1);
temppath = zeros(num,1);
framesz = 100;
activity = cell(time_limit/framesz,1);
shelfinfo1 = cell(time_limit/framesz,1);
shelfinfo2 = cell(time_limit/framesz,1);
ukm = 1;
activation_time = [];
conmap_area = zeros(time_limit,1);
avoidedpath = zeros(time_limit, 1);
deposit = zeros(time_limit,4);
invol0 = 0;
outvol0 = 0;
q = 0;

station = [1 1;1 graphsz(2); graphsz(1) 1; graphsz(1) graphsz(2)];
scolors = [1:item_width];
if demand == 0
    schances = ones(1,item_width);
elseif demand == 1
    schances = gppdf(1:item_width,0,1,0);
    schances = schances./sum(schances);
end
% schances = flip(schances / sum(schances));

clusternum = 1;
endval = 0;
g = 1;
q = 1;
w = 1;
for n = 1:(graphsz(1)-6)
    clusternum = 1 + (endval * 10);
    for k = 1:(graphsz(2)-5)
        if (mod(k,3) ~= 0 && mod(n,4) ~= 0 && k ~= graphsz(2))
            fieldgraph(n+3,k+3) = 1;
            shelfcluster(g) = clusternum;
            if item_scatter == 0
                shelfcontent(g) = 0;
            elseif item_scatter == 1
                shelfcontent(g) = randsample(scolors, 1, true, schances);
            end
            shelfloc(g,:) = [n+3 k+3];
            tloc(q,w) = g;
            if q == 18
                w = w+1;
                q = 0;
            end
            q = q + 1;
            g = g+1;
        elseif k ~= graphsz(2)
            clusternum = clusternum + 1;
        end
    end
end

if item_scatter == 1
    shelfcontent(randperm(360,360-fill_limit)) = 0;
end

intlim = linspace(0,60,item_width + 1);
shelfcluster = discretize(shelfcluster, intlim, 'IncludedEdge', 'right');
radii = zeros(1, item_width);
radii(item_width) = 26; 
if demand == 1
    temps = flip(schances);
    for n = 1:item_width-1
        area(n) =360.*sum(temps(1:n));
    end
elseif demand == 0
    area = (360)./item_width.*(1:(item_width-1));
end

radii = (-1+sqrt(1+4.*(area/2)))./2;
shelfcluster = ones(360,1).';
g = 1;
cent = [9.5 10.5];
for n=1:size(tloc,1)
    for m=1:size(tloc,2)
        dist(g) = sum(abs(cent - [n m]),2);
        g = g+1;
    end
end
for u = item_width-1:-1:1
    shelfcluster(dist<=round(radii(u))) = item_width-u+1;
end

if dist_dist == 2
    ndist = (1:item_width).*(sum(abs(station(1,:)-round(graphsz/2)),2)/item_width);
elseif dist_dist == 3
    ndist = zeros(1,item_width);
end
rgraph = fieldgraph.*(num+1);

% dist_dist 3
queue = zeros(qsize,1);

indices = 1:length(shelfcontent);
shuffled_indices = indices(randperm(length(indices)));
shelfcontent = shelfcontent(shuffled_indices);
shelfloc = shelfloc(shuffled_indices,:);
shelfcluster = shelfcluster(shuffled_indices);

% agent init
for n = 1:num
    % random placement
    agv(n).grid = station(ceil(n/(num/4)),:);
    agv(n).loc = [(agv(n).grid(1) - 0.5) * cellsz (agv(n).grid(2) - 0.5) * cellsz];
    agv(n).step = 1;
    agv(n).pstep = agv(n).step;
    agv(n).dir = 0; % 1 up 2 down 3 left 4 right
    agv(n).turncnt = 1;
    agv(n).role = 0; % 0 to empty 1 to target 2 to station
    agv(n).evade = 0; %0 off 1 on 2 threshold met
    agv(n).wait = 0;
    agv(n).threshold = rr(n);
    agv(n).color = 'c';
    agv(n).carry = [];
    agv(n).station = randi([1 4]);
    agv(n).t0 = 0;
    agv(n).tf = 0;

    if dist_dist == 0 | dist_dist == 2 | dist_dist == 3
        agv(n).item = randsample(scolors, 1, true, schances);
    elseif dist_dist == 1
        while true
            agv(n).item = randsample(scolors, 1, true, schances);
            emptyshelf = find(shelfcluster == agv(n).item & shelfcontent == 0);
            if ~isempty(emptyshelf)
                break;
            end
        end
    end
    queue(n) = agv(n).item;
    qcount = n;

    if item_scatter == 0
        if dist_dist == 0 | dist_dist == 2 | dist_dist == 3
            emptyshelf = find(shelfcontent == 0);
        end
        distances = sum(abs(shelfloc(emptyshelf,:)-agv(n).grid),2);

        if dist_dist == 2
            randist = (26/4).*randn() + ndist(agv(n).item);
            distances = abs(distances - randist);
        elseif dist_dist == 3
            distances = abs(distances - truncated_normal(31./2,(31/4)*(200/size(find(queue),1))));
        end

        [~, minIndex] = min(distances);
        agv(n).carry = emptyshelf(minIndex);
        shelfcontent(agv(n).carry) = agv(n).item + 100;
        destination(n,:) = shelfloc(agv(n).carry,:);
    elseif item_scatter == 1
        agv(n).role = 0;
        emptyshelf = find(shelfcontent ~= 0);
        targetindices = find(shelfcontent ~= 0 & shelfcontent < 100);
        distances = sum(abs(shelfloc(targetindices,:)-agv(n).grid),2);

        if dist_dist == 2 | dist_dist == 3
            randist = (26/26).*randn() + ndist(agv(n).item);
            distances = abs(distances - randist);
        end

        [~, minIndex] = min(distances);
        agv(n).carry = targetindices(minIndex);
        shelfcontent(agv(n).carry) = agv(n).item + 100;
        destination(n,:) = shelfloc(agv(n).carry,:);
    end

    startpos(n,:) = agv(n).grid;

    indgraph{n} = fieldgraph;
    indgraph{n}(destination(n,1),destination(n,2)) = 3;

    prevdest(n,:) = destination(n,:);
    evdest(n,:) = destination(n,:);

    [shortestPath{n}, activation] = findPath(indgraph{n},agv(n).grid,destination(n,:));

    if agv(n).step < size(shortestPath{n},1)
        agv(n).nextgrid = [shortestPath{n}(agv(n).step+1,1) shortestPath{n}(agv(n).step+1,2)];
    end
    nextloc = [(agv(n).grid(1) - 0.5) * cellsz (agv(n).grid(2) - 0.5) * cellsz];
    agv(n).short = [linspace(agv(n).loc(1),nextloc(1),speed); linspace(agv(n).loc(2),nextloc(2),speed)].';
    agv(n).shortcount = 1;
end

if vis == 1
    writerObj = VideoWriter('myVideo.avi');
    writerObj.FrameRate = 30;
    open(writerObj);
    figure(1);
    hold on;
    axis([0-margin graphsz(1)*cellsz+margin 0-margin graphsz(2)*cellsz+margin]);

    for n = 1:graphsz(1)
        for k = 1:graphsz(2)
            x = (n - 1) * cellsz;
            y = (k - 1) * cellsz;
            % rectangle('Position', [x, y, cellsz, cellsz], 'EdgeColor', 'black','LineWidth',.1);
            if (n == 1 && k == 1) || (n == 1 && k == graphsz(2)) || (n == graphsz(1) && k == 1) || (n == graphsz(1) && k == graphsz(2))
                rectangle('Position', [x, y, cellsz, cellsz], 'FaceColor','b');
            end
        end
    end

    for n = 1:size(shelfcontent,2)
        if shelfcontent(n) == 0 | shelfcontent(n) >= 100
            shelfvis(n) = rectangle('Position', [(shelfloc(n,1) - 1) * cellsz, (shelfloc(n,2) - 1) * cellsz, cellsz, cellsz], 'EdgeColor','none','FaceColor','none','LineWidth',.1);
        else
            shelfvis(n) = rectangle('Position', [(shelfloc(n,1) - 1) * cellsz, (shelfloc(n,2) - 1) * cellsz, cellsz, cellsz],'EdgeColor','none', 'FaceColor',C(shelfcontent(n),:),'LineWidth',.1);
        end
    end

    for n = 1:num
            cellcheck(n) = rectangle('Position',[(agv(n).grid(1) - 1) * cellsz, (agv(n).grid(2) - 1) * cellsz, cellsz, cellsz],'EdgeColor','none','FaceColor','y','LineWidth',.1);
            cellcheck2(n) = rectangle('Position',[(shortestPath{n}(agv(n).step,1) - 1) * cellsz, (shortestPath{n}(agv(n).step,2)- 1) * cellsz, cellsz, cellsz],'EdgeColor','none','FaceColor','m','LineWidth',.1);
            plt(n) = plot((agv(n).grid(1) - 0.5) * cellsz+ r*cos(t),(agv(n).grid(2) - 0.5) * cellsz + r*sin(t),'color',agv(n).color,'LineWidth',2);
            path(n) = plot((shortestPath{n}(:,1)-0.5)*cellsz, (shortestPath{n}(:,2)-0.5)*cellsz,'color','b','LineWidth',2);
            agvnum(n) = text(agv(n).loc(1),agv(n).loc(2),num2str(n),'HorizontalAlignment','center');
    end
    kp = 0;
    nuc = text(0,0-10,num2str(k),'HorizontalAlignment','center');
end

while k < time_limit
    if size(find(shelfcontent ~= 0),2) < fill_limit
        construction = k;
    end
    if transition ~= 0 & transition == k
        schances = flip(schances);
    end
    for n = 1:num
        % agv moved to nextgrid
        if agv(n).shortcount >= speed
            % agv update step, nextgrid
            agv(n).pstep = agv(n).step;
            if agv(n).step > 1
                rgraph(shortestPath{n}(agv(n).step-1,1), shortestPath{n}(agv(n).step-1,2)) = 0;
            end
            agv(n).grid = shortestPath{n}(agv(n).step,:);
            heatmap(agv(n).grid(1),agv(n).grid(2)) = heatmap(agv(n).grid(1),agv(n).grid(2)) + 1;

            agentmap = zeros(graphsz);
            gridCoordinates = cat(1, agv.grid);
            linearIndices = sub2ind(graphsz, gridCoordinates(:, 1), gridCoordinates(:, 2));
            agentmap(linearIndices) = 1;

            if size(shortestPath{n},1) > agv(n).step
                nextcell = rgraph(shortestPath{n}(agv(n).step+1,1),shortestPath{n}(agv(n).step+1,2));
                if (nextcell == 0 || nextcell == n || nextcell == (num + 1)) || (coloredstation == 2 && ismember([shortestPath{n}(agv(n).step+1,1),shortestPath{n}(agv(n).step+1,2)], station, 'rows'))
                    agv(n).step = agv(n).step+1;
                    agv(n).nextgrid = [shortestPath{n}(agv(n).step,1) shortestPath{n}(agv(n).step,2)];
                    agv(n).evade = 0;
                else
                    collisionct(n) = collisionct(n) + 1;
                    % head on detection
                    if nextcell~= num+1 && (size(shortestPath{n},1) > agv(n).step) && (size(shortestPath{nextcell},1) > agv(nextcell).step)
                        nextgrid1 = [shortestPath{n}(agv(n).step+1,1), shortestPath{n}(agv(n).step+1,2)];
                        nextgrid2 = [shortestPath{nextcell}(agv(nextcell).step+1,1), shortestPath{nextcell}(agv(nextcell).step+1,2)];
                        if all(nextgrid2 == agv(n).grid) && all(nextgrid1 == agv(nextcell).grid)
                            if agv(n).role <  agv(nextcell).role
                                agv(n).evade = 1;
                            elseif (agv(n).role == agv(nextcell).role) && (agv(n).threshold > agv(nextcell).threshold)
                                agv(n).evade = 1;
                            end
                        elseif (agv(n).role == agv(nextcell).role) && agv(nextcell).evade ~= 0
                                agv(n).evade = 1;
                        elseif agv(n).wait > 5
                            congestionmap(agv(n).grid(1), agv(n).grid(2)) = congestionmap(agv(n).grid(1), agv(n).grid(2)) + 1;
                            agv(n).evade = 1;
                        end
                    end
                end
            end

            % turn detection
            turndir = agv(n).nextgrid - agv(n).grid;
            if agv(n).dir == 1 || agv(n).dir == 2
                if turndir(2) == 0
                    agv(n).turncnt = turnspeed;
                end
            elseif agv(n).dir == 3 || agv(n).dir == 4
                if turndir(1) == 0
                    agv(n).turncnt = turnspeed;
                end
            end
            if turndir(1) > 0
                agv(n).dir = 4;
            elseif turndir(1) < 0
                agv(n).dir = 3;
            elseif turndir(2) > 0
                agv(n).dir = 1;
            elseif turndir(2) < 0
                agv(n).dir = 2;
            end

            % evasion
            if agv(n).evade == 1
                evasionct(n) = evasionct(n) + 1;
                free = findfreecells(rgraph,agv(n).grid(1),agv(n).grid(2),agv(n).role,num,1);
                if ~isempty(free)
                    agv(n).step = 1;
                    shortestPath{n} = [agv(n).grid; free(randi([1 size(free,1)]),:)];
                    agv(n).step = agv(n).step+1;
                    agv(n).nextgrid = [shortestPath{n}(agv(n).step,1) shortestPath{n}(agv(n).step,2)];
                    agv(n).evade = 2;
                end
            elseif agv(n).evade == 2
                indgraph{n} = fieldgraph;
                originalgraph{n} = fieldgraph;
                if pathrecon == 2
                    recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) = recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) - 1;
                    [row, col] = find(recongraph > pathdensity);
                    for tta = 1:size(row,1)
                        indgraph{n}(row(tta),col(tta)) = 2;
                    end
                    indgraph{n}(shortestPath{n}(agv(n).step-1,1), shortestPath{n}(agv(n).step-1,2)) = 3;
                elseif pathrecon == 1
                    recongraph = fieldgraph;
                    congraph = congestion_detection(agentmap, kernel_sizes, thresholds);
                    recongraph(congraph > 0 & recongraph == 0) = congraph(congraph > 0 & recongraph == 0) + 1;
                    [row, col] = find(recongraph > 1);
                    for tta = 1:size(row,1)
                        indgraph{n}(row(tta),col(tta)) = 2;
                    end
                    indgraph{n}(shortestPath{n}(agv(n).step-1,1), shortestPath{n}(agv(n).step-1,2)) = 3;
                elseif pathrecon == 3
                    indgraph{n}(shortestPath{n}(agv(n).step-1,1), shortestPath{n}(agv(n).step-1,2)) = 3;
                end

                indgraph{n}(agv(n).grid(1),agv(n).grid(2)) = 0;
                indgraph{n}(destination(n,1),destination(n,2)) = 5;
                originalgraph{n}(agv(n).grid(1),agv(n).grid(2)) = 0;
                originalgraph{n}(destination(n,1),destination(n,2)) = 5;
    
                [shortestPath{n}, activation] =  findPath(indgraph{n},agv(n).grid,destination(n,:));

                [originalPath{n}, ~] =  findPath(originalgraph{n},agv(n).grid,destination(n,:));

                agv(n).step = 1;
                agv(n).evade = 0;
            end

            % reset destination
            if all(agv(n).grid == destination(n,:))
                if pathrecon == 2
                    recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) = recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) - 1;
                elseif pathrecon == 1
                    recongraph = fieldgraph;
                    congraph = congestion_detection(agentmap, kernel_sizes, thresholds);
                    recongraph(congraph > 0 & recongraph == 0) = congraph(congraph > 0 & recongraph == 0) + 1;
                end

                if agv(n).role == 0 % return shelf
                    rgraph(shelfloc(agv(n).carry,1),shelfloc(agv(n).carry,2)) = num+1;

                    if size(find(shelfcontent ~= 0),2) < fill_limit
                        distances = sum(abs(station-agv(n).grid),2);
                        [~, minIndex] = min(distances);
                        agv(n).station = minIndex;
    
                        destination(n,:) = [station(agv(n).station,1) station(agv(n).station,2)];
                        agv(n).role = 2;
                    else
                        clear targetindices;
                        clear minIndex;
                        clear distances;
                        while true
                            targetindices = find(~ismember(shelfloc, agv(n).grid, 'rows') & shelfcontent.' == randsample(scolors, 1, true, schances));
                            if ~isempty(targetindices)
                                break;
                            end
                        end
                        distances = sum(abs(shelfloc(targetindices,:)-agv(n).grid),2);
                        [~, minIndex] = min(distances);
                        agv(n).nt = targetindices(minIndex);
    
                        agv(n).role = 1;
                        destination(n,:) = shelfloc(agv(n).nt,:);
                        shelfcontent(agv(n).nt) = shelfcontent(agv(n).nt) + 100;
                    end

                    shelfcontent(agv(n).carry) = agv(n).item;
                    if vis == 1
                        kp = 1;
                        svis = agv(n).carry;
                        fc = C(shelfcontent(agv(n).carry),:);
                    end

                    agv(n).carry = 0;
                    agv(n).item = 0;
                    startpos(n,:) = agv(n).grid;

                elseif agv(n).role == 1 % reach target
                    agv(n).carry = agv(n).nt;
                    agv(n).item = shelfcontent(agv(n).nt)-100;
                    shelfcontent(agv(n).nt) = 0;
                    agv(n).nt = 0;
                    rgraph(shelfloc(agv(n).carry,1),shelfloc(agv(n).carry,2)) = 0;
                    if vis == 1
                        kp = 1;
                        svis = agv(n).carry;
                        fc = "none";
                    end

                    distances = sum(abs(station-agv(n).grid),2);
                    [~, minIndex] = min(distances);
                    agv(n).station = minIndex;

                    destination(n,:) = [station(agv(n).station,1) station(agv(n).station,2)];
                    agv(n).role = 2;

                    startpos(n,:) = agv(n).grid;

                elseif agv(n).role == 2% reach station
                    station_in(agv(n).station) = station_in(agv(n).station)+1;
                    station_out(agv(n).station) = station_out(agv(n).station)+1;
                    clear stationshelves;
                    clear validIndices;
                    clear status1_indices;
                    clear mdist;
                    agv(n).carry = 0;
                    agv(n).item = 0;

                    if dist_dist == 0 | dist_dist == 2 | dist_dist == 3
                        agv(n).item = randsample(scolors, 1, true, schances);
                        emptyshelf = find(shelfcontent == 0);
                    elseif dist_dist == 1
                        while true
                            agv(n).item = randsample(scolors, 1, true, schances);
                            emptyshelf = find(shelfcontent == 0 & shelfcluster == agv(n).item);
                            if ~isempty(emptyshelf)
                                break
                            end
                        end
                    end

                    distances = sum(abs(shelfloc(emptyshelf,:)-agv(n).grid),2);
                    item_in(agv(n).item) = item_in(agv(n).item) + 1;
                    if queue(end) == 0
                        queue(qcount) = agv(n).item;
                    else
                        queue = circshift(queue,1);
                        queue(end);
                        queue(end) = agv(n).item;
                    end
                    qcount = qcount + 1;
                    token_counts = [size(find(queue==1),1), size(find(queue==2),1), size(find(queue==3),1), size(find(queue==4),1)];
                    deposit(k,:) = token_counts;

                    % dist_dist 3
                    if dist_dist == 3
                        [stc, stcI] = sort(token_counts);
                        stc(stc==0) = .01;
                        ndist = lmax - ((lmax)./stc)./(sum(1./stc));
                        % ndist = 26.*flip((token_counts-min(token_counts))./(max(token_counts)-min(token_counts))) + 5;
                        % ndist = 26.* (sum(token_counts)./token_counts./sum(sum(token_counts)./token_counts)) + 5;
                        sdev = lmax - ((lmax-precis)./qsize).*size(find(queue),1);
                        randist = sdev.*rand+ndist(flip(stcI) == agv(n).item);
                        if randist > lmax
                            randist = lmax;
                        elseif randist < 5
                            randist = 5;
                        end
                        % randist = truncated_normal(ndist(flip(stcI) == agv(n).item),(precis.*26/26)*(200/size(find(queue),1)));
                        distances = abs(distances - randist);
                    elseif dist_dist == 2
                        randist = (precis).*randn() + ndist(agv(n).item);
                        distances = abs(distances - randist);
                    end

                    [~, minIndex] = min(distances);
                    agv(n).carry = emptyshelf(minIndex);
                    shelfcontent(agv(n).carry) = agv(n).item + 100;
                    destination(n,:) = shelfloc(agv(n).carry,:);

                    startpos(n,:) = agv(n).grid;

                    if vis == 1
                        kp = 1;
                        svis = agv(n).carry;
                        fc = "none";
                    end
                    
                    agv(n).tf = k;
                    picktime{n} = [picktime{n} (agv(n).tf - agv(n).t0)];
                    agv(n).t0 = k;

                    random_pair = [temppath(n); k];  
                    if isempty(totalpath{n})
                        totalpath{n} = random_pair;
                    else
                        totalpath{n} = [totalpath{n} random_pair];
                    end

                    agv(n).role = 0;
                    temppath(n) = 0;
                end

                indgraph{n} = fieldgraph;
                originalgraph{n} = fieldgraph;

                if pathrecon == 2
                    [row, col] = find(recongraph > pathdensity);
                    for tta = 1:size(row,1)
                        indgraph{n}(row(tta),col(tta)) = 2;
                    end
                elseif pathrecon == 1
                    [row, col] = find(recongraph > 1);
                    for tta = 1:size(row,1)
                        indgraph{n}(row(tta),col(tta)) = 2;
                    end
                end

                indgraph{n}(agv(n).grid(1),agv(n).grid(2)) = 0;
                indgraph{n}(destination(n,1),destination(n,2)) = 5;
                originalgraph{n}(agv(n).grid(1),agv(n).grid(2)) = 0;
                originalgraph{n}(destination(n,1),destination(n,2)) = 5;

                [shortestPath{n}, activation] =  findPath(indgraph{n},agv(n).grid,destination(n,:));

                if activation == 1
                    activation_time = [activation_time k];
                else
                    avoidedpath(k) = size(shortestPath{n},1);
                end

                if pathrecon == 2
                    recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) = recongraph(sub2ind(size(recongraph), shortestPath{n}(:,1),shortestPath{n}(:,2))) + 1;
                elseif pathrecon == 1
                    recongraph = fieldgraph;
                    congraph = congestion_detection(agentmap, kernel_sizes, thresholds);
                    recongraph(congraph > 0 & recongraph == 0) = congraph(congraph > 0 & recongraph == 0) + 1;
                end

                if pathrecon == 1
                    [conarea,~] = find(congraph~=0);
                    conmap_area(k) = size(conarea,1);
                elseif pathrecon == 2
                    [conarea,~] = find(indgraph{n}==2);
                    conmap_area(k) = size(conarea,1);
                end

                [originalPath{n}, ~] =  findPath(originalgraph{n},agv(n).grid,destination(n,:));

                switch agv(n).role
                    case 0
                        toshelf_cell{n} = [toshelf_cell{n}, size(shortestPath{n},1)];
                        original_toshelf_cell{n} = [original_toshelf_cell{n}, size(originalgraph{n},1)];
                    case 1
                        totarget_cell{n} = [toshelf_cell{n},  size(shortestPath{n},1)];
                        original_totarget_cell{n} = [original_totarget_cell{n},  size(originalgraph{n},1)];
                    case 2
                        tostation_cell{n} = [toshelf_cell{n},  size(shortestPath{n},1)];
                        original_tostation_cell{n} = [original_tostation_cell{n},  size(originalgraph{n},1)];
                end

                agv(n).step = 1;
            end

            rgraph(agv(n).nextgrid(1),agv(n).nextgrid(2)) = n;
            rgraph(agv(n).grid(1),agv(n).grid(2)) = n;

            nextloc = [(agv(n).nextgrid(1)- 0.5) * cellsz (agv(n).nextgrid(2) - 0.5) * cellsz];
            agv(n).short = [linspace(agv(n).loc(1),nextloc(1),speed); linspace(agv(n).loc(2),nextloc(2),speed)].';
            agv(n).shortcount = 1;

            if agv(n).pstep == agv(n).step
                agv(n).wait = agv(n).wait + 1;
            else
                temppath(n) = temppath(n) + 1;
                agv(n).wait = 0;
            end
        end
        
        % turn wait
        if agv(n).turncnt > 1
            agv(n).turncnt = agv(n).turncnt - 1;
        end

        % short move
        agv(n).loc = agv(n).short(agv(n).shortcount,:);
        if agv(n).turncnt <= 1
            agv(n).shortcount = agv(n).shortcount+1;
        end
        if vis == 1
            if agv(n).evade ~= 0
                agv(n).color = 'r';
            else
                agv(n).color = 'c';
            end
            set(cellcheck(n), 'Position', [(agv(n).grid(1) - 1) * cellsz, (agv(n).grid(2) - 1) * cellsz, cellsz, cellsz]);
            set(cellcheck2(n), 'Position', [(agv(n).nextgrid(1) - 1) * cellsz, (agv(n).nextgrid(2)- 1) * cellsz, cellsz, cellsz]);
            set(plt(n), 'XData', agv(n).loc(1) + r*cos(t), 'YData', agv(n).loc(2) + r*sin(t),'color',agv(n).color);
            set(path(n), 'XData', (shortestPath{n}(:,1)-0.5)*cellsz, 'YData',(shortestPath{n}(:,2)-0.5)*cellsz,'color','b');
            set(nuc, 'String', num2str(k));
            set(agvnum(n), 'Position',[agv(n).loc(1) agv(n).loc(2)]);
            if kp == 1
                set(shelfvis(svis),'Facecolor',fc);
                kp = 0;
            end
            writeVideo(writerObj, getframe(gcf));
        end
        
    end
    if vis == 1
        pause(.001);
    end
    if mod(k,2000) == 0
        q = q + 1;
        invol = sum(station_in,1);
        outvol = sum(station_out,1);
        inot(q) = invol;
        outot(q) = outvol;
        inrate(q) = (invol-invol0)/2000;
        outrate(q) = (outvol-outvol0)/2000;
        invol0 = invol;
        outvol0 = outvol;
    elseif mod(k,framesz) == 0
        map = zeros(graphsz);
        x = cellfun(@(x) x(1), {agv.grid});
        y = cellfun(@(x) x(2), {agv.grid});
        indices = sub2ind(size(map), x, y);
        map(indices) = 1;
        activity{ukm} = map;
        shelfinfo1{ukm} = shelfloc;
        shelfinfo2{ukm} = shelfcontent;
        ukm = ukm+1;
    end
    k = k + 1;
    
end
close(writerObj);
% end

% Functions

function [distances, sorted_labels] = calculate_distances(token_counts)    
    labels = [1 2 3 4];
    
    [sorted_counts, sort_index] = sort(token_counts, 'descend');
    sorted_labels = labels(sort_index);
    
    N = sum(sorted_counts);
    
    f = sorted_counts / N;
    
    F = cumsum(f);
    
    distances = zeros(size(sorted_counts));
    for i = 1:length(sorted_counts)
        if i == 1
            distances(i) = F(i) / 2;
        else
            distances(i) = (F(i-1) + F(i)) / 2;
        end
    end
end

function sample = truncated_normal(m, s)
    lower_bound = 5;
    upper_bound = 31;
    a = (lower_bound - m) / s;
    b = (upper_bound - m) / s;
    alpha = normcdf(a);
    beta = normcdf(b);
    u = rand() * (beta - alpha) + alpha;
    sample = norminv(u, m, s);
    sample = max(lower_bound, min(upper_bound, sample));
end