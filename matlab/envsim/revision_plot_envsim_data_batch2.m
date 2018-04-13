% script to plot simulation data
close all; clear; clc;

data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111133039.5-40percent.data');
data2 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111134526.8-40percent.data');
data3 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111142548.12-40percent.data');

res_range5 = data1(1:500,5);
res_range8 = data2(1:500,5);
res_range12 = data3(160:659,5);

res_mean = [mean(data1(:,5)) mean(data2(:,5)) mean(data3(:,5))]
res_max = [max(data1(:,5)) max(data2(:,5)) max(data3(:,5))]

cnt_range5 = 0;
cnt_range8 = 0;
cnt_range12 = 0;
threshold = 0.1;
for i = 1:500
    if res_range5(i) > threshold
        cnt_range5 = cnt_range5 + 1;
    end
    if res_range8(i) > threshold
        cnt_range8 = cnt_range8 + 1;
    end
    if res_range12(i) > threshold
        cnt_range12 = cnt_range12 + 1;
    end
end
cnt_range5
cnt_range8
cnt_range12

edges = [-0.05 0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5]*100;

figure
%plot(data1(1:500,5),'-d')
%xlim([0, 500])
%ylim([0, 0.6])
histogram(data1(1:500,5)*100,edges)
ylim([0, 120])
title('Path repair result - sensor range 5')
xlabel('Percentage of path cost reduction')
ylabel('Number of simulation instances')

figure
%plot(data2(1:500,5),'-d')
%xlim([0, 500])
%ylim([0, 0.6])
histogram(data2(1:500,5)*100,edges)
ylim([0, 120])
title('Path repair result - sensor range 8')
xlabel('Percentage of path cost reduction')
ylabel('Number of simulation instances')

figure
%plot(data3(160:660,5),'-d')
%xlim([0, 500])
%ylim([0, 0.6])
histogram(data3(160:660,5)*100,edges)
ylim([0, 120])
title('Path repair result - sensor range 12')
xlabel('Percentage of path cost reduction')
ylabel('Number of simulation instances')