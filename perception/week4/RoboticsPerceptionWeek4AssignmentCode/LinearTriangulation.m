function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     K  - size (3 x 3) camera intrsinc parameter for both cameras
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

% Y = [0 -1 0;1 0 0;0 0 1];
% F   = EstimateFundamentalMatrix(x1,x2);
% E   = EssentialMatrixFromFundamentalMatrix(F,K);
% [U,D,V] = svd(E);
t1 = -R1*C1;
P1 = K*[R1 t1];
t2 = -R2*C2;
P2 = K*[R2 t2];

% compute 2 point correspondences for each input point
for i = 1:length(x1(:,1))
    x_1 = [x1(i,1);x1(i,2);1];
    x_2 = [x2(i,1);x2(i,2);1];
    s1  = Vec2Skew(x_1);
    s2  = Vec2Skew(x_2);
    A   = [s1*P1;s2*P2];
    [u,d,v] = svd(A);
    tmp = v(:,end) / v(end,end);
    X(i,:) = tmp(1:3)';
end

