% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
map = myMap;
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,
 
    % Find grids hit by the rays (in the gird map coordinate)
    for i = 1:length(ranges(:,j))
        % use (y,x) convention
        yocc = -ranges(i,j)*sin(scanAngles(i)+pose(3,j)) + pose(2,j);
        xocc =  ranges(i,j)*cos(scanAngles(i)+pose(3,j)) + pose(1,j);
        % grid position
        yind(i) = ceil( myResol * yocc ) + myorigin(2);
        xind(i) = ceil( myResol * xocc ) + myorigin(1);
        
        % Find occupied-measurement cells and free-measurement cells
        % get cells in between
        xorig = myorigin(1) + ceil(myResol * pose(1,j));
        yorig = myorigin(2) + ceil(myResol * pose(2,j));
        
        [freex, freey] = bresenham(xorig,yorig,xind(i),yind(i));
        % convert to 1d index
        free = sub2ind(size(myMap),freey,freex);
        % set end point value
        map(yind(i),xind(i)) = 1;
        % set free cell values
        map(free) = 0;
    end
    
    occ = find(map > 0);
    free = setdiff(1:param.size(1)*param.size(2),occ);
    myMap(free) = myMap(free) - lo_free;
    myMap(occ)  = myMap(occ) + lo_occ;

    % Saturate the log-odd values
    over_min = find(myMap < lo_min);
    over_max = find(myMap > lo_max);
    myMap(over_min) = lo_min;
    myMap(over_max) = lo_max;

    % Visualize the map as needed
    %image(myMap)
   

end

end

