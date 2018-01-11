% script to plot simulation data
close all; clear; clc;

%data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111133039.5-40percent.data');
%data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111134526.8-40percent.data');
data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111142548.12-40percent.data');

res_range5 = data1(1:500,5);

res_mean = mean(data1(:,5))
res_max = max(data1(:,5))

cnt_range5 = 0;
threshold = 0.1;
for i = 1:500
    if res_range5(i) > threshold
        cnt_range5 = cnt_range5 + 1;
    end
end

figure
plot(data1(1:500,5),'-d')
xlim([0, 500])
ylim([0, 0.4])
title('path repair result - sensor range 5')
xlabel('simulation instance')
ylabel('percentage of path length reduction')
