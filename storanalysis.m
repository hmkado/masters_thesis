clc
clear all
close all

tvol = readmatrix("svol.csv");
tcol = readmatrix("scol.csv");

[mean(tvol,2) std(tvol,0,2)]

[mean(tcol,2) std(tcol,0,2)]

[p1,tbl1,stats1]  = anova1(tvol.');
xticklabels({'Rand','AS-CBS'});
[p2,tbl2,stats2]  = anova1(tcol.');
xticklabels({'Rand','AS-CBS'});

%%
multcompare(stats1);

