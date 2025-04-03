clc
clear all
close all

trial_no = 3;   % 3:5.10  5:5.9a  6:5.9b 
dataA = readtable(sprintf("trial_%d/collisionct.csv",trial_no), 'ReadVariableNames', false);
dataB = readtable(sprintf("trial_%d/completion.csv",trial_no), 'ReadVariableNames', false);
histA = [];
histB = [];
totalA = 0;
totalB = 0;
dataA.Properties.VariableNames = {'Unit', 'Time', 'Event'};
dataB.Properties.VariableNames = {'Unit', 'Time', 'Event'};

dataA = sortrows(dataA, {'Unit', 'Time'});
dataB = sortrows(dataB, {'Unit', 'Time'});

units = unique(dataA.Unit);
resultsA = cell(length(units), 1);
resultsB = cell(length(units), 1);
avgIntervalsA = zeros(length(units), 1);
avgIntervalsB = zeros(length(units), 1);
for i = 1:length(units)
    unit_dataA = dataA(dataA.Unit == units(i), :);
    unit_dataB = dataB(dataB.Unit == units(i), :);
    
    intervalsA = diff([0; unit_dataA.Time]);
    intervalsB = diff([0; unit_dataB.Time]);
    
    unit_resultsA = table(unit_dataA.Time, intervalsA, unit_dataA.Event, ...
        'VariableNames', {'Time', 'Interval', 'Event'});
    unit_resultsB = table(unit_dataB.Time, intervalsB, unit_dataB.Event, ...
        'VariableNames', {'Time', 'Interval', 'Event'});
    
    resultsA{i} = unit_resultsA;
    resultsB{i} = unit_resultsB;
    
    histA = [histA; size(resultsA{i},1)];
    histB = [histB; size(resultsB{i},1)];
    totalA = totalA + size(resultsA{i},1);
    totalB = totalB + size(resultsB{i},1);
    avgIntervalsA(i) = mean(intervalsA);
    avgIntervalsB(i) = mean(intervalsB);
    fprintf('Unit %d:\n', units(i));
    fprintf('Average interval for A: %.4f\n', avgIntervalsA(i));
    fprintf('Average interval for B: %.4f\n', avgIntervalsB(i));
    fprintf('\n');
end

figure(1);
bar([histA histB])
ylim([0 max([histA; histB])+1])
avgA = totalA / length(units);
avgB = totalB / length(units);
xlabel(['Robot Number']);
xlabel(['Counts']);
legend(['Collision','Completion']);

fprintf('Overall average interval for A: %.4f\n', mean(avgIntervalsA));
fprintf('Overall average interval for B: %.4f\n', mean(avgIntervalsB));
