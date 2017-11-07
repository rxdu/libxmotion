close all;clear;clc;

imu_log_file = '/home/rdu/CarLog/raw_imu.20171107120247.data';
mag_log_file = '/home/rdu/CarLog/raw_mag.20171107120247.data';

imu_data = csvread(imu_log_file);
mag_data = csvread(mag_log_file);

t_imu = imu_data(:,1);
tb_imu = t_imu(1);
t_imu = t_imu - tb_imu;

t_mag = mag_data(:,1);
tb_mag = t_mag(1);
t_mag = t_mag - tb_mag;

figure(1)
hold on
title('gyro')
plot(t_imu, imu_data(:,2),'r')
plot(t_imu, imu_data(:,3),'g')
plot(t_imu, imu_data(:,4),'b')
legend('x','y','z')

figure(2)
hold on
title('accel')
plot(t_imu, imu_data(:,5),'r')
plot(t_imu, imu_data(:,6),'g')
plot(t_imu, imu_data(:,7),'b')
legend('x','y','z')

figure(3)
hold on
title('mag')
plot(t_mag, mag_data(:,2),'r')
plot(t_mag, mag_data(:,3),'g')
plot(t_mag, mag_data(:,4),'b')
legend('x','y','z')

