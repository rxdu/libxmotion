% cleanup workspace
close all;clear;clc;

% load IMU data file
log_file = "/home/rdu/Workspace/balancing_robot/data/imu/balancing_beagle.20181218144805.data"
[utime,raw_accel,raw_gyro,raw_magn] = read_imu_log(log_file);
utime = utime/1000;
dlen = size(utime,1);

% % kalman filter
% q0 = quaternion(1,0,0,0);
% P = eye(4) * 0.5;
% T = 0.005;
%  
% qhat = zeros(dlen+1);
% qhat(1) = q0;
% for tk = 1:2%dlen
%     yw = [raw_gyro(tk,1), raw_gyro(tk,2), raw_gyro(tk,3)]*T/2;
%     qyw = quaternion(0, yw(1), yw(2), yw(3));
%     qyw = qyw/norm(qyw);
%     qhat_tk_minus = qhat(tk);
%     qhat_tk_prior = quatmultiply(qhat_tk_minus, qyw*T/2);
%     %F_tkm1 = quaternion(0, 
% end

% plot raw data
plot_imu_data(utime, raw_accel, raw_gyro, raw_magn)
