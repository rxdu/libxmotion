
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
            const = posDes(1, ineqConst.start(s, 1), ineqConst.dim(s, i)) * ...
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




