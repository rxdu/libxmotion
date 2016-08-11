close all;
clear;
clc;

% define 3 points
start = [0; 0; 0; 0 ]
middle = [0.2; 0.6; 0.8; pi/8]
goal = [1.0; 1.0; 1.0; pi/4]

% plot points
px = [start(1), middle(1), goal(1)]
py = [start(2), middle(2), goal(2)]
pz = [start(3), middle(3), goal(3)]

plot3(px,py,pz, '-*')


