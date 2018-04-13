% script to plot simulation data
close all; clear; clc;

%data = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/prsim.20171205114104.data');
data = csvread('/home/rdu/Workspace/librav/data/log/quad/prsim/batch2/prsim.20180111165734.cmp.data');
data = data(1:400,:);

run1_data = [];
run2_data = [];
for i = 1:size(data,1)
   if data(i,2) == 1 
       run1_data = [run1_data; data(i,:)];
   else
       run2_data = [run2_data; data(i,:)];
   end
end

traveled_dist1 = run1_data(:,3) - run1_data(:,5);
traveled_dist2 = run2_data(:,3) - run2_data(:,5);

mean1 = mean(traveled_dist1)
mean2 = mean(traveled_dist2)

count = 0;
for i = 1:min(size(run1_data(:,6)),size(run2_data(:,6)))
   if  traveled_dist1(i) <= traveled_dist2(i)
       count = count + 1;
   end
end
count

% figure
% plot(run1_data(:,6),'-rd')
% hold on
% plot(run2_data(:,6),'-bd')
% legend('path repair seed path', 'shortest seed path')
% xlim([0, 200])
% ylim([0, 0.4])
% xlabel('simulation instance')
% ylabel('percentage of path length reduction')

figure
hold on
%run_error = (run2_data(:,4) - run2_data(:,5)) - (run1_data(:,4) - run1_data(:,5));
run_error = (traveled_dist2 - traveled_dist1)./traveled_dist1;
%plot(run_error,'-rd')
%plot(zeros(size(run1_data(:,6))),'b')
%xlim([0, 200])
%ylim([0, 0.4])
edges = [-0.1 -0.05 0 0.05 0.1 0.15 0.2]*100;
histogram(run_error*100,edges)
title('Path repair result - sensor range 5')
xlabel('Percentage of path cost reduction')
ylabel('Number of simulation instances')


