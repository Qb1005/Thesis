clear all
clc
% straight line w_ref = 0
% curve line w_ref = v_ref / R
R = 0.2;
v_ref = 1.5;
w_ref = v_ref/R;
%w_ref = 0;
X0 = [0 0 0];
A = [0 w_ref 0;
     -w_ref 0 v_ref;
     0 0 0];
B = [1 0;
     0 0;
     0 1];
C = [0 1 1];
D = 0;

Q = [1 0 0;
     0 40 0;
     0 0 28];
R = 0.01*[1 0;
       0 1];
   
K = lqr(A,B,Q,R);
P = care(A,B,Q,R);

Acl = A - B * K;
sys = ss(Acl,B,C,D);

figure(1);
step(sys);
title("Step input");