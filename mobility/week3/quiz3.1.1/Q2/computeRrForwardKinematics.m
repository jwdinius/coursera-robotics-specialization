function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]
elbow = [cos(rads1),sin(rads1)];
endeff =[cos(rads1) + cos(rads1+rads2),sin(rads1) + sin(rads1+rads2)];
