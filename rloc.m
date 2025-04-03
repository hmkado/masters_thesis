clc;
clear all;
close all;

rloc = struct2cell(load('rloc.mat'));
rloc = rloc{1};

figure(2);
colormap_turbo = turbo;
new_colormap = [1 1 1; colormap_turbo];
for n = 1:length(rloc)
    data = rloc{n};
    data(data>0) = data(data>0) - 4;
    heatmap(data, 'Colormap', new_colormap);
    pause(1);
end