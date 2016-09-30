% 7/31/13
% desiredTraj.m
% generates an optimal trajectory through a set of keyframes
% an implementation of techniques described in "Minimum Snap Trajectory Generation 
% and Control for Quadrotors", Mellinger and Kumar 
%
% indices convention: 
% for a polynominal of order n, its coefficients are:
%       x(t) = c_n t^n + c_[n-1] t^(n-1) + ... + c_1 t + c_0
% for m keyframes, the times of arrival at keyframes are t0, t1, ..., tm
% the polynominal segment between keyframe 0 and 1 is x1, 1 and 2 is x2,
%   ... m-1 and m is xm
%
% Dependencies: findTraj.m, plotTraj.m, findTrajCorr.m, evaluateTraj.m
%   findContConstraints.m, findFixedConstraints.m, findDerivativeCoeff.m, findCostMatrix.m

close all
clear all
clc
%echo on

diary ~/output.txt

%%%
% set up problem
r = 4; %derivative to minimize in cost function
n = 7; %order of desired trajectory
m = 2;
d = 2; %dimensions

% specify the m+1 keyframes
%tDes = [0;1.2; 3; 5; 6]; % %specify desired arrival times at keyframes
tDes = [0;1.2;3];
% specify desired positions and/or derivatives at keyframes
posDes(:, :, 1) = [-0.3 0.5 1.05; 0 Inf 0; 0 Inf 0; 0 0 0; 0 0 0; 0 0 0];
posDes(:, :, 2) = [1.15 1 1.5; 0 Inf 0; 0 Inf 0; 0 0 0; 0 0 0; 0 0 0];
%posDes(:, :, 3) = [2.25 2.25 2.25; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0];
[i, j, k] = size(posDes);
l = length(tDes);

% specify s corridor constraints
ineqConst.numConst = 2; %integer, number of constraints 
ineqConst.start = [1; 2]; %sx1 matrix of keyframes where constraints begin
ineqConst.dim = [1 2; 1 2]; %sxd matrix of dimensions that each constraint applies to

ineqConst.nc = 20; %sx1 matrix of numbers of intermediate points
ineqConst.delta = 0.05; %sx1 matrix of maximum distnaces

%%%
% verify that the problem is well-formed
% polynominal trajectories must be at least of order 2r-1 to have all derivatives lower than r defined
if (n < (2*r-1)) 
    error('trajectory is not of high enough order for derivative optimized')
end

if (i < r),
    error('not enough contraints specified: to minimize kth derivative, constraints must go up to the (k-1)th derivative');
end

if (j < m+1 || l < m+1), % must specify m+1 keyframes for m pieces of trajectory
    error('minimum number of keyframes not specified');
end
% 
% if (ismember(Inf, posDes(:, 1, :)) || ismember(Inf, posDes(:, m+1, :)) )
%     error('endpoints must be fully constrained');
% end

if (k < d)
    error('not enough dimensions specified');
end

%%% 
% find trajectories for each dimension, nondimensionalized in time
% xT holds all coefficents for all trajectories
% row i is the ith coefficient for the column jth trajectory in dimension k
xT = zeros(n+1, m, d); 
posDes_opt = zeros(r, m+1, d); 
xT2 = zeros(n+1, m, d); 
xT3 = zeros(n+1, m, d); 
% for i = 1:d,
%   xT(:, :, i) = find_polynomial(r, n, m, i, tDes, posDes);
% end

global A_ineq
global b_ineq

xT = find_corridor_poly(r, n, m, d, tDes, posDes, ineqConst);

fprintf([repmat('%f\t', 1, size(A_ineq, 2)) '\n'], A_ineq')

for i = 1:d,
   fprintf('coefficients of dimension %i\n', i)
   xT(:, :, i)
end

% plot QP traj
dimLabels{1} = 'x (m)';
dimLabels{2} = 'y (m)'; 
dimLabels{3} = 'z (m)'; 
plotTraj(0, tDes(m+1), xT, n, m, d, tDes, posDes, 0.01, dimLabels, [], 2*r);
