% script to plot simulation data
close all; clear; clc;

data = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/prsim.20171026130113.cmp.data');

run1_data = [];
run2_data = [];
for i = 1:size(data,1)
   if data(i,2) == 1 
       run1_data = [run1_data; data(i,:)];
   else
       run2_data = [run2_data; data(i,:)];
   end
end

mean1 = mean(run1_data(:,6))
mean2 = mean(run2_data(:,6))

figure
plot(run1_data(:,6),'-rd')
hold on
plot(run2_data(:,6),'-d')

%xlim([0, 500])
%ylim([0, 0.4])
%title('path repair result - sensor range 8')
%xlabel('simulation instance')
%ylabel('percentage of path length reduction')

