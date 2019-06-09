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
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N    % for each time,
       
      pose_t = pose(:, j);
      x = pose_t(1);
      y = pose_t(2);
      theta = pose_t(3);
      
      % Convert to grid index for the robot (within myMap)
      xx = ceil(myResol*x) + myorigin(1);
      yy = ceil(myResol*y) + myorigin(2);
      
      range_set = ranges(:, j);
      measCnt = length(range_set);
            
      % Find grids hit by the rays (in the gird map coordinate)
      for i=1:measCnt
          
          % Find occupied-measurement cells and free-measurement cells
          % Calculate real location
          Xocc = range_set(i)*cos(theta + scanAngles(i)) + x;
          Yocc = -1*range_set(i)*sin(theta + scanAngles(i)) + y;
          
          % Convert to grid index of this occupied cell
          Dxy = ceil(myResol* [Xocc Yocc]') + myorigin;
          
          % Convert subscripts (i.e. Dxy) to linear indices in myMap
          iDxy = sub2ind(size(myMap), Dxy(2), Dxy(1));
          
          % Find the locations of the free cells between the robot origin
          % and the occupied cell
          [freex, freey] = bresenham(xx, yy, Dxy(1), Dxy(2));

          free = sub2ind(size(myMap), freey, freex);
          
          % Update the log-odds
          myMap(free) = myMap(free) - lo_free;              
          myMap(iDxy) = myMap(iDxy) + lo_occ;
          
          % Saturate the log-odd values
          myMap(free) = max(myMap(free), lo_min);
          myMap(iDxy) = min(myMap(iDxy), lo_max);          
      end
      
end


