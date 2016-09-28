% 7/30/13
% findTrajCorr.m
% generate optimal trajectory in many dimensions
%   this allows for specification of corridor constraints
% Dependencies: findContConstraints.m, findFixedConstraints.m,
%   findDerivativeCoeff.m, findCostMatrix.m
%
% inputs:
%   r: integer, derivative to minimize in cost function
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   tDes: (m+1) x 1 vector, desired times of arrival at keyframes
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   ineqConst: structure of sx1 arrays, for s constraints, with elements:
%       numConst: integer, number of constraints
%       start: sx1 matrix, keyframes where constraints begin
%       delta: sx1 matrix, maximum distance
%       nc: sx1 matrix, number of intermediate points
%       dim: sxd matrix, the d dimensions that the constraint applies to
% outputs:
%   xT: (n+1) x m xd matrix, where row i contains the ith coefficient for
%       the jth trajectory in dimension k
%       xT is nondimensionalized in time

function [xT] = find_corridor_poly(r, n, m, d, tDes, posDes, ineqConst)

    % use nondimensionalized time
    t0 = 0;
    t1 = 1;

    % construct cost matrix Q
    Q_opt = [];

    for dim = 1:d,    
        % construct cost matrix Q
        Q_joint = [];
        for i = 1:m,
            Q = get_hessian(n, r, t0, t1); % find cost matrix for each segment
            %1/((tDes(i+1, 1)-tDes(i, 1))^(2*r-1))
            Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*r-1)).*Q;% multiply by time factor to nondimensionalize

            Q_joint = blkdiag(Q_joint, Q);%put in block diagonal matrix
        end

        % put each dimension's Q_joint into a block diagonal matrix
        Q_opt = blkdiag(Q_opt, Q_joint);
    end

    %%%
    % construct equality constraints
    A_opt = [];
    b_opt = [];

    for dim = 1:d,

        % construct fixed value constraints
        [A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes);

        % put each A_eq for each dimension into block diagonal matrix
        A_opt = blkdiag(A_opt, A_eq);
        b_opt = [b_opt; b_eq];

    end

    % construct any inequality constraints
    [A_ineq, b_ineq] = constructCorrConstraints(n, m, d, posDes, ineqConst, t0, t1);
    %size(A_ineq)
    %size(b_ineq)

    % find optimal trajectory through quadratic programming
    xT_all = quadprog(Q_opt,[],A_ineq, b_ineq,A_opt,b_opt);

    % explicitly break trajectory into its piecewise parts and dimensions for output
    xT = zeros((n+1), m, d);
    for dim = 1:d,
        thisxT = xT_all((dim-1)*(n+1)*m+1:dim*(n+1)*m);
        for j = 1:m,
            xT(:, j, dim) = thisxT((j-1)*(n+1)+1:j*(n+1));
        end
    end

end


% make corridor constraints into inequality constraints
%
% inputs: 
%   n: integer, order of desired trajectory
%   m: integer, number of pieces in trajectory
%   d: integer, number of dimensions
%   posDes: r x m x d matrix, desired positions and/or derivatives at keyframes,
%       Inf represents unconstrained values
%       each row i is the value the (i-1)th derivative of column j for
%       dimenison k
%   ineqConst: structure of sx1 arrays, for s constraints, with elements:
%       numConst: integer, number of constraints
%       start: sx1 matrix, keyframes where constraints begin
%       delta: sx1 matrix, maximum distance
%       nc: sx1 matrix, number of intermediate points
%       dim: sxd matrix, the d dimensions that the constraint applies to
%   t0: real value, begnning time of the trajectory
%   t1: real value, end time of the trajectory
% outputs: 
%   A_ineq, b_ineq: matrices in formulation A_ineq x <= b_ineq
function [A_ineq, b_ineq] = constructCorrConstraints(n, m, d, posDes, ineqConst, t0, t1)

    A_ineq = [];
    b_ineq = [];

    for s = 1:ineqConst.numConst, % for each constraint

        % find distance between keyframes
        pos1 = [];
        pos2 = [];
        for i = 1:length(ineqConst.dim(s, :))
            pos1 = [pos1 posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))];
            pos2 = [pos2 posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))];
        end
        R = pdist([pos1; pos2]);

        % for each dimension to optimize
        for i = 1:length(ineqConst.dim(s, :)),

            % find constant to add to b term of inequality
            const = posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))*...
                ((posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i)))^2/R^2-1);

            for j = 1:length(ineqConst.dim(s, :)),
                if i ~= j,
                    const = const + ...
                        posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j)) * ...
                        (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j))) * ...
                        (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i)))/R^2;
                end
            end

            % find coefficients for constraint equation
            coeff = zeros(length(ineqConst.dim(s, :)), 1);
            for j = 1:length(ineqConst.dim(s, :))
                if i == j,
                    coeff(j, 1) = (1 - ...
                        (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j)))^2/R^2);
                else
                    coeff(j, 1) = -(posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, j))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, j))) ...
                        * (posDes(1, ineqConst.start(s, 1)+1, ineqConst.dim(s, i))-posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i))) ...
                        /R^2;
                end
            end
            coeff

            % find 2 constraints for each intermediate point
            A_temp = zeros(2*ineqConst.nc, (n+1)*m*d);
            b_temp = zeros(2*ineqConst.nc, 1);

            for j = 1:ineqConst.nc,
                intT = t0+ j/(1+ineqConst.nc)*(t1-t0);
                %find all non-zero coefficients in this row of A_temp
                terms = zeros(length(ineqConst.dim(s, :)), n+1);

                for k = 1:length(ineqConst.dim(s, :))
                    for l = 0:n,
                        terms(k, l+1) = intT^(n-l)*coeff(k, 1);
                    end
                    
                    A_temp((j-1)*2+1, ...
                        (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1 ...
                        : (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1+n) ...
                        = terms(k, :);
                    A_temp(j*2, ...
                        (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1 ...
                        : (ineqConst.dim(s, k)-1)*m*(n+1) + (ineqConst.start(s, 1)-1)*(n+1) + 1+n) ...
                        = -terms(k, :);

                end

                b_temp((j-1)*2+1, 1) = ineqConst.delta-const;
                b_temp(j*2, 1) = ineqConst.delta+const;
            end

            A_ineq = [A_ineq; A_temp];
            b_ineq = [b_ineq; b_temp];

        end

    end
end




