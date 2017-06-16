function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 3) pose rotation
A = zeros(3*length(X(:,1)),12);
for i = 1:length(X(:,1))
    XtildeT = [X(i,:) 1];
    u       = x(i,1);
    v       = x(i,2);
    ind     = 3*(i-1);
    A(ind+1:ind+3,:) = [zeros(1,4) -XtildeT v*XtildeT;
                        XtildeT zeros(1,4) -u*XtildeT;
                        -v*XtildeT u*XtildeT zeros(1,4)];
end

[~,~,V] = svd(A);

P(1,:) = V(1:4,end);
P(2,:) = V(5:8,end);
P(3,:) = V(9:12,end);

KinvP  = K \ P;
Rp = KinvP(:,1:3);

[U,D,V] = svd(Rp);

if (det(U*V') > 0)
    R = U*V';
    t = KinvP(:,4) / D(1,1);
    C = -R'*t;
else
    R = -U*V';
    t = -KinvP(:,4) / D(1,1);
    C = -R'*t;    
end
