close all;clear;clc;

data_still = csvread('/home/rdu/Workspace/auto_racing/data/log/raw_imu.20171030155831.car_still.data');
data_driving = csvread('/home/rdu/Workspace/auto_racing/data/log/raw_imu.20171030160549.car_driving.data');

figure(1)
hold on
title('gyro (still)')
plot(data_still(:,1),'r')
plot(data_still(:,2),'g')
plot(data_still(:,3),'b')
legend('x','y','z')

figure(2)
hold on
title('accel (still)')
plot(data_still(:,4),'r')
plot(data_still(:,5),'g')
plot(data_still(:,6),'b')
legend('x','y','z')

figure(3)
hold on
title('gyro (driving)')
plot(data_driving(:,1),'r')
plot(data_driving(:,2),'g')
plot(data_driving(:,3),'b')
legend('x','y','z')

figure(4)
hold on
title('accel (driving)')
plot(data_driving(:,4),'r')
plot(data_driving(:,5),'g')
plot(data_driving(:,6),'b')
legend('x','y','z')