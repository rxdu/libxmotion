clear;
clc; 

s = [2.56435e-13
0.000124413
   0.902115
  0.0977604
  1.58332e-07]

trans = [0.455     0     0     0     0;
0.545 0.455     0     0     0;
    0 0.545 0.455     0     0;
    0     0 0.545 0.455     0;
    0     0     0 0.545 0.455]

s'*trans^2