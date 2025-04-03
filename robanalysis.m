clc;
clear all;
close all;

tvol = readmatrix("tvol.csv");
tcol = readmatrix("tcol.csv");

[mean(tvol,2) std(tvol,0,2)]

[mean(tcol,2) std(tcol,0,2)]

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

[p1,tbl1,stats1]  = anova1(tvol.');
xticklabels({'None','Agent','LRA*','WHCA*'});
[p2,tbl2,stats2]  = anova1(tcol.');
xticklabels({'None','Agent','LRA*','WHCA*'});

%%
multcompare(stats1);
