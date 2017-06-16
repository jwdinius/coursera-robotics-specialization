% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

%rand('seed',11011);

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 750;                           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
w = 1/M*ones(M,1);
C = [1 -5;-5 10];              % correlation scores
corr = zeros(M,1);
Q = diag([.1^2;.1^2;.1^2]);
invalid_particle = 0;
uninitialized = 1;
wmax = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
Prs = P;
wrs = w;
%load('practice.mat','pose');
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    %fprintf('err = %f, %f, %f, w = %f\n',myPose(1,j-1)-pose(1,j-1),myPose(2,j-1)-pose(2,j-1),myPose(3,j-1)-pose(3,j-1),wmax);
    % 1) Propagate the particles
    for k = 1:M
        if (uninitialized)
            P(3,k) = 2*pi*(rand-1);
            if (k == M)
                uninitialized = 0;
            end
        else
            %v        = sqrt((Q(1,1)*randn)^2 + (Q(2,2)*randn)^2);
            %v        = sqrt(Q(1,1) + Q(2,2))*rand;
%             P(1,k)   = P(1,k) + v*cos(P(3,k));
%             P(2,k)   = P(2,k) - v*sin(P(3,k));
%             P(3,k)   = P(3,k) + sqrt(Q(3,3))*randn;
            P(:,k) = P(:,k) + chol(Q)*randn(3,1);
        end
        free = [];
        occ  = [];
        
        % 2) Measurement Update
        for i = 1:length(ranges(:,j))
%             fprintf('%d, %d, %d\n',j,i,k);
            %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
            % use (y,x) convention
            yocc = -ranges(i,j)*sin(scanAngles(i)+P(3,k)) + P(2,k);
            xocc =  ranges(i,j)*cos(scanAngles(i)+P(3,k)) + P(1,k);
            % grid position
            yind = ceil( myResolution * yocc ) + myOrigin(2);
            xind = ceil( myResolution * xocc ) + myOrigin(1);
            
            % Find occupied-measurement cells and free-measurement cells
            % get cells in between
            xorig = myOrigin(1) + ceil(myResolution * P(1,k));
            yorig = myOrigin(2) + ceil(myResolution * P(2,k));
            if ( xind < 1 | xind > length(map(1,:)) | ...
                    yind < 1 | yind > length(map(:,1)) )
                invalid_particle = 1;
            else
                occ  = [occ;sub2ind(size(map),yind,xind)];
            end
        end
        if (~invalid_particle)
            %   2-2) For each particle, calculate the correlation scores of the particles
            occ_occ   = intersect(find(map > 0.6), unique(occ));
            corr(k)   = length(occ_occ) * C(2,2);
        else
            corr(k) = 0;
            invalid_particle = 0;
        end
    end
    
    %   2-3) Update the particle weights
    wc = w.*corr;
    s  = sum(wc);
    w = 1/s*wc;
    Meff = floor(sum(w)^2 / sum(w.^2));
%    fprintf('%f\n',sum(w));
%     [ws,ind] = sort(w,'descend');
%     fprintf('%f, %f, %f, %f, %f\n',ws(1),ws(2),ws(3),ws(4),ws(5));
    %   2-4) Choose the best particle to update the pose
    [~,I] = max(w);
    %fprintf('%f\n',wmax);
    myPose(:,j) = P(:,I);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    if Meff < .2*M
        %fprintf('resampling...\n');
        c = cumsum(w);
        for i = 1:M
            u = rand;
            I = min(find(u <= c));
            Prs(:,i) = P(:,I);
        end
        P = Prs;
        w = 1/M*ones(M,1);
    end
    % 4) Visualize the pose on the map as needed
%     plot(myPose(1,j)*param.resol+param.origin(1), ...
%     myPose(2,j)*param.resol+param.origin(2), 'r.-');%%Predicted pose
%     plot(pose(1,j)*param.resol+param.origin(1), ...
%     pose(2,j)*param.resol+param.origin(2), 'y.-');%%Actual pose
%     drawnow;
%     colormap('gray');
%     axis equal;
end
end

