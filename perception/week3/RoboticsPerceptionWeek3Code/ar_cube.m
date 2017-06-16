function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
h1 = H(:,1);
h2 = H(:,2);
h3 = cross(h1,h2);
Rp = [h1,h2,h3];
[U,S,V] = svd(Rp);
tmp = eye(3);
tmp(3,3) = det(U*V');
R = U*tmp*V';
t = H(:,3) / norm(h1);
% YOUR CODE HERE: Project the points using the pose
proj_points = zeros(length(render_points(:,1)),2);
for i = 1:length(render_points(:,1))
    X = render_points(i,:)';
    Xc = K*(R*X+t);
    proj_points(i,1:2) = [Xc(1)/Xc(3) Xc(2)/Xc(3)];
end

end
