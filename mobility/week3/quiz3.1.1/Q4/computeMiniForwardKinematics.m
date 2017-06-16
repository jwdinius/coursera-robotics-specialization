function [endeff] = computeMiniForwardKinematics(rads1,rads2)

alpha = .5*(rads1+rads2)+pi;
beta  = .5*(rads1-rads2);

% right triangle of interest: (r+L1*cos(beta))^2 + L1*sin(beta)^2 = L2^2
r = -cos(beta)+sqrt(cos(beta)^2+3);

endeff = [r*cos(alpha),r*sin(alpha)];