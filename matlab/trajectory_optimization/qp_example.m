% Define QP parameters
H = diag([1; 0]);
f = [3; 4];
A = [-1 -3; 2 5; 3 4];
b = [-15; 100; 80];
l = zeros(2,1);

% Set quadprog options
% R2011a or later
options = optimset('Algorithm','interior-point-convex');
% Construct the QP, invoke solver
[x,fval] = quadprog(H,f,A,b,[],[],l,[],[],options);
% Extract optimal value and solution
disp(x); % 0
disp(fval); % 5