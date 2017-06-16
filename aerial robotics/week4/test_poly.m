close; clear; clc

waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';

n = length(waypoints(1,:));
traj_time = 4*linspace(0,1,n);
waypoints0 = waypoints;
np = 7;
ns = length(waypoints(:,1));
Ax = zeros((np+1)*(n-1));
bx = zeros((np+1)*(n-1),1);
trat = 1; % trajectory times deliberately chosen to enforce this

% organize as follows:
%     [pos x0
%      vel x0
%      acc x0
%      jerk x0
%      snap x0
%      ...
%      ...
%      ...
%      pos x1
%      ...
bx(1) = waypoints0(1,1);

% for t = 0
Ax(1,1) = 1;
Ax(2,2) = 1;
Ax(3,3) = 1;
Ax(4,4) = 1; % first 3 derivatives are zero
bx(2:4) = 0;
rowoff = 4;

for i = 1:n-1
    % x component
    rowind = rowoff+(i-1)*(np+1)+1;
    colbeg = rowind-rowoff;
    colend = colbeg + np;
    Ax(rowind,colbeg:colend) = 1;
    bx(rowind) = waypoints0(1,i+1); %p_i(S_i) = w_(i+1)
    if (i < n-1)
        rowind = rowind + 1;
        Ax(rowind,colbeg:colend) = 1;
        Ax(rowind,colend+1) = -1; % pos smoothing = sum_k alpha_ik - alpha_(i+1)0
        rowind = rowind+1;
        colbeg = colbeg+1;
        Ax(rowind,colbeg:colend) = 1:7;
        Ax(rowind,colend+2) = -trat; % vel smoothing
        rowind = rowind + 1;
        colbeg = colbeg + 1;
        Ax(rowind,colbeg:colend) = [1 3 6 10 15 21];
        Ax(rowind,colend+3) = -trat^2; % acc smoothing
        rowind = rowind + 1;
        colbeg = colbeg + 1;
        Ax(rowind,colbeg:colend) = [1 4 10 20 35];
        Ax(rowind,colend+4) = -trat^3;
        rowind = rowind + 1;
        colbeg = colbeg + 1;
        Ax(rowind,colbeg:colend) = [1 5 15 35];
        Ax(rowind,colend+5) = -trat^4;
        rowind = rowind + 1;
        colbeg = colbeg + 1;
        Ax(rowind,colbeg:colend) = [1 6 21];
        Ax(rowind,colend+6) = -trat^5;
        rowind = rowind + 1;
        colbeg = colbeg + 1;
        Ax(rowind,colbeg:colend) = [1 7];
        Ax(rowind,colend+7) = -trat^6;
%         rowind = rowind + 1;
%         colbeg = colbeg + 1;
%         Ax(rowind,colbeg:colend) = 1;
%         Ax(rowind,colend+8) = -trat^7;
    end
    
end
%don't double-count p_(n-1)=endpt
rowind = rowind + 1;
colbeg = (np+1)*(n-2)+2;
Ax(rowind,colbeg:end) = [1 2 3 4 5 6 7];
rowind = rowind + 1;
colbeg = colbeg + 1;
Ax(rowind,colbeg:end) = [1 3 6 10 15 21];
rowind = rowind + 1;
colbeg = colbeg + 1;
Ax(rowind,colbeg:end) = [1 4 10 20 35];

alphax = inv(Ax)*bx;

t = linspace(0,4,100);

for i = 1:length(t)
    if (t(i) <= traj_time(2))
        dt = t(i)-traj_time(1);
        T  = traj_time(2)-traj_time(1);
        dt = dt / T;
        x(i) = alphax(1:8)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
    elseif (t(i) <= traj_time(3))
        dt = t(i)-traj_time(2);
        T  = traj_time(3)-traj_time(2);
        dt = dt / T;
        x(i) = alphax(9:16)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
    elseif (t(i) <= traj_time(4))
        dt = t(i)-traj_time(3);
        T  = traj_time(4)-traj_time(3);
        dt = dt / T;
        x(i) = alphax(17:24)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
    else
        dt = t(i)-traj_time(4);
        T  = traj_time(5)-traj_time(4);
        dt = dt / T;
        x(i) = alphax(25:32)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
    end
        
end

plot(t,x)
