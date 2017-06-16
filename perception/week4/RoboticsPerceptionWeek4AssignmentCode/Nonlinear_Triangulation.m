function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix for both
%     cameras
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
X = zeros(length(X0(:,1)),3);
for i = 1:length(X0(:,1))
    inp = X0(i,:);
    inpx1 = x1(i,:);
    inpx2 = x2(i,:);
    inpx3 = x3(i,:);
    
    tmp = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, inpx1', inpx2', inpx3', inp');
    X(i,:) = tmp';
end
end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
b   = [x1(1);x1(2);x2(1);x2(2);x3(1);x3(2)];
xp1 = K*R1*(X0-C1);
xp2 = K*R2*(X0-C2);
xp3 = K*R3*(X0-C3);
f   = [xp1(1)/xp1(3);
       xp1(2)/xp1(3);
       xp2(1)/xp2(3);
       xp2(2)/xp2(3);
       xp3(1)/xp3(3);
       xp3(2)/xp3(3)];
e = b - f;
iter = 0;
while iter < 20
     J = [Jacobian_Triangulation(C1, R1, K, X0);
          Jacobian_Triangulation(C2, R2, K, X0);
          Jacobian_Triangulation(C3, R3, K, X0)];
     JTJ  = J'*J;
     delX = JTJ \ J' * e;
     X0   = X0 + delX;
     xp1 = K*R1*(X0-C1);
     xp2 = K*R2*(X0-C2);
     xp3 = K*R3*(X0-C3);
     f   = [xp1(1)/xp1(3);
            xp1(2)/xp1(3);
            xp2(1)/xp2(3);
            xp2(2)/xp2(3);
            xp3(1)/xp3(3);
            xp3(2)/xp3(3)];
     e = b - f;
     norm(e)
     iter = iter + 1;
end
X = X0;
end

function dfdX = Jacobian_Triangulation(C, R, K, X)
xp = K*R*(X-C);
f  = K(1,1);
px = K(1,3);
py = K(2,3);
u = xp(1);
v = xp(2);
w = xp(3);
dudX = [f*R(1,1)+px*R(3,1) f*R(1,2)+px*R(3,2) f*R(1,3)+px*R(3,3)];
dvdX = [f*R(2,1)+py*R(3,1) f*R(2,2)+py*R(3,2) f*R(2,3)+py*R(3,3)];
dwdX = [R(3,1) R(3,2) R(3,3)];
dfdX = [(w*dudX - u*dwdX) / w^2;
        (w*dvdX - v*dwdX) / w^2];
end
