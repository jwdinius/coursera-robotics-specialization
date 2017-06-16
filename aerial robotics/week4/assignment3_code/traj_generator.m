function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     n = length(waypoints(1,:));
%     d0 = 8*linspace(0,1,n);
%     traj_time = cumsum(d0);
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
% end
%


%% Fill in your code here

persistent waypoints0 traj_time alphax alphay alphaz
if nargin > 2
    n = length(waypoints(1,:));
    traj_time = 12*linspace(0,1,n);
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
    
    alphax = Ax \ bx;

    Ay = zeros((np+1)*(n-1));
    by = zeros((np+1)*(n-1),1);
    
    by(1) = waypoints0(2,1);
    
    % for t = 0
    Ay(1,1) = 1;
    Ay(2,2) = 1;
    Ay(3,3) = 1;
    Ay(4,4) = 1; % first 3 derivatives are zero
    by(2:4) = 0;
    rowoff = 4;
    
    for i = 1:n-1
        % y component
        rowind = rowoff+(i-1)*(np+1)+1;
        colbeg = rowind-rowoff;
        colend = colbeg + np;
        Ay(rowind,colbeg:colend) = 1;
        by(rowind) = waypoints0(2,i+1); %p_i(S_i) = w_(i+1)
        if (i < n-1)
            rowind = rowind + 1;
            Ay(rowind,colbeg:colend) = 1;
            Ay(rowind,colend+1) = -1; % pos smoothing = sum_k alpha_ik - alpha_(i+1)0
            rowind = rowind+1;
            colbeg = colbeg+1;
            Ay(rowind,colbeg:colend) = 1:7;
            Ay(rowind,colend+2) = -trat; % vel smoothing
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Ay(rowind,colbeg:colend) = [1 3 6 10 15 21];
            Ay(rowind,colend+3) = -trat^2; % acc smoothing
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Ay(rowind,colbeg:colend) = [1 4 10 20 35];
            Ay(rowind,colend+4) = -trat^3;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Ay(rowind,colbeg:colend) = [1 5 15 35];
            Ay(rowind,colend+5) = -trat^4;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Ay(rowind,colbeg:colend) = [1 6 21];
            Ay(rowind,colend+6) = -trat^5;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Ay(rowind,colbeg:colend) = [1 7];
            Ay(rowind,colend+7) = -trat^6;
            %         rowind = rowind + 1;
            %         colbeg = colbeg + 1;
            %         Ay(rowind,colbeg:colend) = 1;
            %         Ay(rowind,colend+8) = -trat^7;
        end
        
    end
    %don't double-count p_(n-1)=endpt
    rowind = rowind + 1;
    colbeg = (np+1)*(n-2)+2;
    Ay(rowind,colbeg:end) = [1 2 3 4 5 6 7];
    rowind = rowind + 1;
    colbeg = colbeg + 1;
    Ay(rowind,colbeg:end) = [1 3 6 10 15 21];
    rowind = rowind + 1;
    colbeg = colbeg + 1;
    Ay(rowind,colbeg:end) = [1 4 10 20 35];
    
    alphay = Ay \ by;

    Az = zeros((np+1)*(n-1));
    bz = zeros((np+1)*(n-1),1);
    
    bz(1) = waypoints0(3,1);
    
    % for t = 0
    Az(1,1) = 1;
    Az(2,2) = 1;
    Az(3,3) = 1;
    Az(4,4) = 1; % first 3 derivatives are zero
    bz(2:4) = 0;
    rowoff = 4;
    
    for i = 1:n-1
        % z component
        rowind = rowoff+(i-1)*(np+1)+1;
        colbeg = rowind-rowoff;
        colend = colbeg + np;
        Az(rowind,colbeg:colend) = 1;
        bz(rowind) = waypoints0(3,i+1); %p_i(S_i) = w_(i+1)
        if (i < n-1)
            rowind = rowind + 1;
            Az(rowind,colbeg:colend) = 1;
            Az(rowind,colend+1) = -1; % pos smoothing = sum_k alpha_ik - alpha_(i+1)0
            rowind = rowind+1;
            colbeg = colbeg+1;
            Az(rowind,colbeg:colend) = 1:7;
            Az(rowind,colend+2) = -trat; % vel smoothing
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Az(rowind,colbeg:colend) = [1 3 6 10 15 21];
            Az(rowind,colend+3) = -trat^2; % acc smoothing
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Az(rowind,colbeg:colend) = [1 4 10 20 35];
            Az(rowind,colend+4) = -trat^3;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Az(rowind,colbeg:colend) = [1 5 15 35];
            Az(rowind,colend+5) = -trat^4;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Az(rowind,colbeg:colend) = [1 6 21];
            Az(rowind,colend+6) = -trat^5;
            rowind = rowind + 1;
            colbeg = colbeg + 1;
            Az(rowind,colbeg:colend) = [1 7];
            Az(rowind,colend+7) = -trat^6;
            %         rowind = rowind + 1;
            %         colbeg = colbeg + 1;
            %         Az(rowind,colbeg:colend) = 1;
            %         Az(rowind,colend+8) = -trat^7;
        end
        
    end
    %don't double-count p_(n-1)=endpt
    rowind = rowind + 1;
    colbeg = (np+1)*(n-2)+2;
    Az(rowind,colbeg:end) = [1 2 3 4 5 6 7];
    rowind = rowind + 1;
    colbeg = colbeg + 1;
    Az(rowind,colbeg:end) = [1 3 6 10 15 21];
    rowind = rowind + 1;
    colbeg = colbeg + 1;
    Az(rowind,colbeg:end) = [1 4 10 20 35];
    
    alphaz = Az \ bz;
    
else
    np = 7;
    if(t > traj_time(end))
        t = traj_time(end);
        t_index = length(traj_time);
    else
        t_index = find(traj_time > t,1);
    end

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    T = traj_time(t_index)-traj_time(t_index-1);
    dt = t / T;
    
    start = (np+1)*(t_index-2)+1;
    endd  = start + np;
    desired_state.pos = [alphax(start:endd)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
                         alphay(start:endd)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7];
                         alphaz(start:endd)'*[1;dt;dt^2;dt^3;dt^4;dt^5;dt^6;dt^7]];
                     
    desired_state.vel = [alphax(start+1:endd)'*[1;2*dt;3*dt^2;4*dt^3;5*dt^4;6*dt^5;7*dt^6];
                         alphay(start+1:endd)'*[1;2*dt;3*dt^2;4*dt^3;5*dt^4;6*dt^5;7*dt^6];
                         alphaz(start+1:endd)'*[1;2*dt;3*dt^2;4*dt^3;5*dt^4;6*dt^5;7*dt^6]]/T;
                     
    desired_state.acc = [alphax(start+2:endd)'*[2;6*dt;12*dt^2;20*dt^3;30*dt^4;42*dt^5];
                         alphay(start+2:endd)'*[2;6*dt;12*dt^2;20*dt^3;30*dt^4;42*dt^5];
                         alphaz(start+2:endd)'*[2;6*dt;12*dt^2;20*dt^3;30*dt^4;42*dt^5]]/T^2;
                                      
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
end

end

