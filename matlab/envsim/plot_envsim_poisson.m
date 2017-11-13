% script to plot simulation data
close all; clear; clc;

data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch_poisson2/prsim.20171113122516.range5.data');
data2 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch_poisson2/prsim.20171113144505.range8.data');
data3 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch_poisson2/prsim.20171113160131.range12.data');

% res_range = data1(:,5);

res_mean = [mean(data1(:,5)) mean(data2(:,5)) mean(data3(:,5))]
res_max = [max(data1(:,5)) max(data2(:,5)) max(data3(:,5))]

% cnt_range = 0;
% threshold = 0.1;
% for i = 1:500
%     if res_range5(i) > threshold
%         cnt_range5 = cnt_range5 + 1;
%     end
% end

figure
hold on
plot(data1(:,5),'-rd')
plot(data2(:,5),'-gd')
plot(data3(:,5),'-bd')
%xlim([0, 500])
%ylim([0, 0.4])
title('path repair result')
xlabel('simulation instance')
ylabel('percentage of path length reduction')
