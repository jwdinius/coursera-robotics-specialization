function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;

eq1 = cos(theta1) + cos(theta1+theta2) - X;
eq2 = sin(theta1) + sin(theta1+theta2) - Y;

sol = solve(eq1,eq2);

rads1=double(sol.theta1(1));
rads2=2*pi+double(sol.theta2(1));