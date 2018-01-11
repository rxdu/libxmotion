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

mean1 = mean(run1_data(:,6))
mean2 = mean(run2_data(:,6))

count = 0;
for i = 1:min(size(run1_data(:,6)),size(run2_data(:,6)))
   if  run1_data(i,6) >= run2_data(i,6)
       count = count + 1;
   end
end
count

figure
plot(run1_data(:,6),'-rd')
hold on
plot(run2_data(:,6),'-bd')
legend('path repair seed path', 'shortest seed path')
xlim([0, 200])
ylim([0, 0.4])
xlabel('simulation instance')
ylabel('percentage of path length reduction')

% figure
% hold on
% plot(run1_data(:,6)-run2_data(:,6),'-rd')
% plot(zeros(size(run1_data(:,6))),'b')
% xlim([0, 250])
%ylim([0, 0.4])
%title('path repair result - sensor range 8')
%xlabel('simulation instance')
%ylabel('percentage of path length reduction')

