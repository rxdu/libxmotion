close all;clear;clc;

imu_log_file = '/home/rdu/CarLog/raw_imu.20171122145342.data'

imu_data = csvread(imu_log_file);

t_imu = imu_data(:,1);
tb_imu = t_imu(1);
t_imu = t_imu - tb_imu;

gyro_data = imu_data(:,2);

%allan_var = calc_allan_variance(gyro_data);
[T,sigma] = allan(gyro_data, 300, 27870);

figure
hold on
% plot(allan_var(:,1),'r')
% plot(allan_var(:,2),'g')
% plot(allan_var(:,3),'b')
plot(sigma,'r')