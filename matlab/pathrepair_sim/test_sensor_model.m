% clear workspace
close all;
clear;
clc;

%% create space for testing
ssensor = create_3d_space([30 30 10]);
ssensoro = add_obstacle_to_space(ssensor, 0.05);

%% sensor model
sensor_info.sensor_idx = [3 3 10];
sensor_info.sensor_pos = ssensoro.voxels{sensor_info.sensor_idx(1),...
            sensor_info.sensor_idx(2), ...
            sensor_info.sensor_idx(3)}.center_pos;
sensor_info.sensor_angle = pi/3;
sensor_info.range = 10.0;
sensor_info.aov_h = pi*5/12;
sensor_info.aov_v = pi/3;
sensor_info.res = pi/36;

sensor_data.sensor_model = init_depth_sensor(sensor_info);
sensor_data.data = [];

%% plot data
display_3d_space(ssensoro,sensor_data)
