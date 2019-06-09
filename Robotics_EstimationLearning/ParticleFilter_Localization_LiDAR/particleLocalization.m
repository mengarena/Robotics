% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

[maxX maxY] = size(map);

% Define threshold for cell status (for correlation score)
maxVal = max(max(map));
minVal = min(min(map));
thres_occupied = minVal + (maxVal-minVal)*0.66;
thres_free = minVal + (maxVal-minVal)*0.33;

thres_resample = 0.8;  % Threshold for resampling

% Standard variation for position and orientation
sigma_x = 0.04;
sigma_y = 0.04;
sigma_theta = 0.03;
sigma_m = [sigma_x sigma_y sigma_theta];

speed = 0.02;  % Robot velocity

% Decide the number of particles, M.
M = 500;

% Create M number of particles
Psample = repmat(myPose(:,1), [1, M]);  % 3xM

W = repmat(1.0/M, [1, M]);

v = VideoWriter('PF_Loc_testing.avi');
%v.FrameRate = 30;
open(v);

fig = figure(1);
fig.Position = [30, 30, maxX + 50, maxY + 50];

imagesc(map); colormap('gray'); axis equal; hold on;
title("Robot Localization: Particle Filter");

for j = 2:N   % You will start estimating myPose from j=2 using ranges(:,2).

     fprintf("Progress: %d/%d\n", j, N);
     
     % 1) Propagate the particles 
     P = Psample;     
     P(1:2,:) = P(1:2,:) + speed .* [cos(P(3, :)); -sin(P(3, :))];
     P = P + randn(3, M) .* sigma_m';

     % Reset weight for each step
     %W = repmat(1.0/M, [1, M]);

     range_set = ranges(:,j);
     
     pCorr = zeros(1, M);  % Correlation score
     
     % 2) Measurement Update 
     % Find grid cells hit by the rays (in the grid map coordinate frame)    
     for i=1:M     % Particles  (each particle has length(range_set) laser measurements)
         x = P(1,i);
         y = P(2,i);
         theta = P(3,i);
                  
         % Grid index
         Xocc = ceil((range_set .* cos(scanAngles + theta) + x)*myResol) + myOrigin(1);
         Yocc = ceil((-1 * range_set .* sin(scanAngles + theta) + y)*myResol) + myOrigin(2);
             
         Xocc = min(Xocc, maxX);
         Xocc = max(Xocc, 1);
         Yocc = min(Yocc, maxY);
         Yocc = max(Yocc, 1);
             
         % Convert subscripts (i.e. Xocc, Yocc) to linear indices in map
         iDxy = sub2ind(size(map), Yocc, Xocc);
         
         % Correlation score for each particle between laser hit and map (Map Registration)
         pCorr(i) = sum(map(iDxy) >= thres_occupied)*10 + sum(map(iDxy) <= thres_free)*(-5);
     end
      
     pCorr = pCorr - min(pCorr);
     
     % Update the particle weights 
     W = W .* pCorr;
     
     Neffective = power(sum(W),2)/sum(W.^2);  % For effective particles

     sumW = sum(W);
     W = W/sumW;
     [maxW maxIdx] = max(W);  % Select the particle with best correlation
     
     % Choose the best particle to update the pose
     myPose(:,j) = P(:,maxIdx);
     
     % 3) Resample if the effective number of particles is smaller than a threshold
     if Neffective < thres_resample*M         
         cumW = cumsum(W);
         Psample = zeros(3, M);
         Wsample = zeros(1, M);
         for i=1:M
             idx = find(rand < cumW, 1);
             Psample(:,i) = P(:,idx);
             Wsample(i) = W(idx);
         end
         % Normalize the weights
         W = Wsample/sum(Wsample); 
     else
         Psample = P;  % No resample, use previous Particles
     end
          
     % 4) Visualize the pose on the map as needed
     if mod(j, 10) == 0   
        %figure(fig);
        % Delete previous plot to save memory and improve performance
        try
            delete(h2);            
        end
        try
            delete(h1);            
        end
        try
            delete(h3);            
        end
        try
            delete(h4);            
        end

        % Occupied cells positions from laser
        XX = (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*myResol + myOrigin(1);
        YY = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*myResol + myOrigin(2);
        
        total_beams = length(XX);
        laser_beams = length(XX(1:2:total_beams));
        
        % Positions of Robot so far
        robotX = ceil(myPose(1,1:j)*myResol) + myOrigin(1);
        robotY = ceil(myPose(2,1:j)*myResol) + myOrigin(2);

        curRobotX = ceil(myPose(1,j)*myResol) + myOrigin(1);
        curRobotY = ceil(myPose(2,j)*myResol) + myOrigin(2);

        % Occupied cell from laser hit
        hold on;
        h1 = plot(XX, YY, 'g.');

        hold on;
        h2 = plot([repmat(curRobotX, laser_beams,1) XX(1:2:total_beams)]', [repmat(curRobotY, laser_beams, 1) YY(1:2:total_beams)]', 'b-');

        hold on;
        h3 = plot(curRobotX, curRobotY, 'yo');
        
        hold on;
        % Robot trajectory
        h4 = plot(robotX(1:j-1), robotY(1:j-1), 'r.-'); 
                
        drawnow;
        pause(0.1);
        writeVideo(v, getframe(gcf));
     end

end

close(v);
