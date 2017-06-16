function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

N = length(x1(:,1));
A = zeros(N,9);
% this amounts to a linear least squares problem Ax = 0, x is composed of
% elements of fundamental matrix.
for i = 1:N
    u = [x1(i,1) x2(i,1)];
    v = [x1(i,2) x2(i,2)];
    A(i,:) = [u(1)*u(2) u(1)*v(2) u(1) v(1)*u(2) v(1)*v(2) v(1) u(2) v(2) 1];
end

% solution is the final column of V matrix from SVD
[~,D,V] = svd(A);

F = reshape(V(:,9),3,3);

% remove smallest singular value of F to enforce rank 2 condition
[U,D,V] = svd(F);
D(3,3) = 0;

% rank 2 F
F = U*D*V';

% normalization condition
F = F / norm(F);
