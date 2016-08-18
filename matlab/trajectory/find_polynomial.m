% 7/30/13
% findTraj.m
% generate optimal trajectory (in one dimension)
%   note that this implies no coridor constraints
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
%
% inputs: 
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   dim: integer, current dimension
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes, 
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
% outputs:
%   xT: (n+1) x m matrix, where row i contains the ith coefficient for the jth trajectory
%       xT is nondimensionalized in time


function [xT] = find_polynomial(r, n, m, dim, tDes, posDes)

% use nondimensionalized time
t0 = 0;
t1 = 1;

% construct cost matrix Q
Q_joint = [];
for i = 1:m,
    Q = get_hessian(n, r, t0, t1); % find cost matrix for each segment
    Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*r-1)).*Q;% multiply by time factor to nondimensionalize
    
    Q_joint = blkdiag(Q_joint, Q)%put in block diagonal matrix
end

% % construct equality constraints 
% [A_fixed, b_fixed] = get_fixedvalue_constraints(r, n, m, dim, posDes, t0, t1, tDes)
% [A_cont, b_cont] = get_continuous_constraints(r, n, m, dim, posDes, t0, t1, tDes)
% 
% % put each A_eq for each dimension into block diagonal matrix
% A_eq = [A_fixed; A_cont];
% b_eq = [b_fixed; b_cont];

[A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes);

% find optimal trajectory through quadratic programming
opts = optimoptions('quadprog','TolFun', 1e-30);
[xT_all, fval] = quadprog(Q_joint,[],[],[],A_eq,b_eq);

xT_all'*Q_joint*xT_all

% explicitly break tracjetory into its piecewise parts for output
xT = zeros((n+1), m);
for j = 1:m,
    xT(:, j) = xT_all((j-1)*(n+1)+1:j*(n+1));
end

end