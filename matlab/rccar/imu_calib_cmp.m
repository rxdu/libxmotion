close all;clear;clc;

% g = 9.80503 m/s^2 at Worcester, MA

imu_log_file = '/home/rdu/CarLog/raw_imu.20171111235018.data'
imu_calib_log_file = '/home/rdu/CarLog/calib_imu.20171113170524.data'


raw_data = csvread(imu_log_file);
calib_data = csvread(imu_calib_log_file);

figure(1)
hold on
title('gyro - raw')
plot(raw_data(:,2),'r')
plot(raw_data(:,3),'g')
plot(raw_data(:,4),'b')
legend('x','y','z')

figure(2)
hold on
title('accel - raw')
plot(raw_data(:,5),'r')
plot(raw_data(:,6),'g')
plot(raw_data(:,7),'b')
legend('x','y','z')

figure(3)
hold on
title('accel - calib')
plot(calib_data(:,2),'r')
plot(calib_data(:,3),'g')
plot(calib_data(:,4),'b')
legend('x','y','z')

figure(4)
hold on
title('gryo - calib')
plot(calib_data(:,5),'r')
plot(calib_data(:,6),'g')
plot(calib_data(:,7),'b')
legend('x','y','z')

