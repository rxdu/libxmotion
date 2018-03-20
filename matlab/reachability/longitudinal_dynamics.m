clear;
clc; 

s0 = 0;
v0 = 8;

v_sw = 7.3;
a_max = 7;

u = 0.1;

s = [];
v = [];
for t = 0:0.001:10
   st = (s0 + (v0^2 + 2 * v_sw * u * t)^(3/2) - v0^3)/(3 * v_sw * u);
   s = [s st];
   vt = sqrt(v0^2 + 2 * v_sw * u * t);
   v = [v vt];
end

