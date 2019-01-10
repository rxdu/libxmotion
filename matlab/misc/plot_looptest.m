close all;
clear;
clc;

loop_normal = "/home/rdu/Workspace/balancing_robot/data/loop_test/expt3/looptest_normal.data"
loop_rt = "/home/rdu/Workspace/balancing_robot/data/loop_test/expt3/looptest_rt.data"

data_normal = csvread(loop_normal);
data_rt = csvread(loop_rt);

figure
plot(data_normal(100:11200),'c')
hold on
plot(data_rt(100:11200), 'r')
legend('normal','rt')
xlabel('sample')
ylabel('loop duration(us)')