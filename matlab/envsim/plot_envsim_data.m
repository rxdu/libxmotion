% script to plot simulation data
close all; clear; clc;

data1 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch1/prsim.20170927074826.5-45percent.data');
data2 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch1/prsim.20170926073337.8-45percent.data');
data3 = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch1/prsim.20170927092831-12-45percent.data');

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

figure
plot(data1(1:500,5),'-d')
xlim([0, 500])
ylim([0, 0.4])
title('path repair result - sensor range 5')
xlabel('simulation instance')
ylabel('percentage of path length reduction')

figure
plot(data2(1:500,5),'-d')
xlim([0, 500])
ylim([0, 0.4])
title('path repair result - sensor range 8')
xlabel('simulation instance')
ylabel('percentage of path length reduction')

figure
plot(data3(160:660,5),'-d')
xlim([0, 500])
ylim([0, 0.4])
title('path repair result - sensor range 12')
xlabel('simulation instance')
ylabel('percentage of path length reduction')