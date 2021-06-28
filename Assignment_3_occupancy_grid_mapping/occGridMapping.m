% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels (Initial position of the robot in the
% map)
myOrigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
num_rays = length(scanAngles);
for j = 1:N % for each time,    
    
    % current x, y, theta of the robot from the pose
    x = pose(1,j);
    y = pose(2,j);
    theta = pose(3,j);
    
    % Locating the position of the robot in the map
    i_x_robot = ceil(myResol * x) + myOrigin(1);
    i_y_robot = ceil(myResol * y) + myOrigin(2);
    
    % Find grids hit by the rays (in the gird map coordinate)
    rays = ranges(:,j);
    x1k = rays.*cos(theta + scanAngles) + x;
    x2k = - rays.*sin(theta + scanAngles) + y;
    
    i1k = ceil(myResol*x1k) + myOrigin(1);
    i2k = ceil(myResol*x2k) + myOrigin(2); 
    
    % Find occupied-measurement cells and free-measurement cells
    %-- % Note: sub2ind - Converting subscripts to linear indices of the matrix
    occ = sub2ind(size(myMap), i2k, i1k); 
    
    free = []
    for k = 1:num_rays
        [ix_free, iy_free] = bresenham(i_x_robot, i_y_robot, i1k(k), i2k(k));
        free = [free; ix_free, iy_free];
    end
    free = sub2ind(size(myMap), free(:,2), free(:,1));
    
    % Update the log-odds
    for i = 1:length(occ)
        myMap(occ(i))= myMap(occ(i)) + lo_occ;
    end
    
    for j = 1:length(free)
        myMap(free(j)) = myMap(free(j)) - lo_free;
    end
    
    % Saturate the log-odd values
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

    % Visualize the map as needed
    imagesc(myMap);

end

end
