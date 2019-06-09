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

thres_resample = 0.75;  % Threshold for resampling

sigma_x = 0.01;
sigma_y = 0.01;
sigma_theta = 0.02;
sigma_m = [sigma_x sigma_y sigma_theta];

speed = 0.02;  % Robot velocity

% Decide the number of particles, M.
M = 200;

% Weights for particles
Wsample = repmat(1.0/M, [1, M]);

% Create M number of particles
Psample = repmat(myPose(:,1), [1, M]);  % 3xM


for j = 2:N   % You will start estimating myPose from j=2 using ranges(:,2).

     fprintf("Progress: %d/%d\n", j, N);
     
     % 1) Propagate the particles 
     P = Psample;     
     P(1:2,:) = P(1:2,:) + speed .* [cos(P(3, :)); -sin(P(3, :))];
     P = P + randn(3, M) .* sigma_m';
     
     %     P = myPose(:, j-1) + noise';   % 3xM

     %dist = 0.02 + randn(2, M)*0.1;
     %P(1:2,:) = P(1:2,:) + dist;
     % myPose(:, j) = P(:,1);
     
     %for i=1:M
     %    P(1,i) = myPose(1,j-1) + sigma_x*randn;
     %    P(2,i) = myPose(2,j-1) + sigma_y*randn;
     %    P(3,i) = myPose(3,j-1) + sigma_theta*randn;         
     %end
     
     W = repmat(1.0/M, [1, M]);
     %W = Wsample;
     range_set = ranges(:,j);
     %measCnt = length(range_set);
     
     pCorr = zeros(1, M);  % Correlation score
     %Dxy = zeros(M, 2);
     
     % 2) Measurement Update 
     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
     for i=1:M     % Particles
         x = P(1,i);
         y = P(2,i);
         theta = P(3,i);
                  
         %for t = 1:measCnt    % Laser measurement
             Xocc = ceil((range_set .* cos(scanAngles + theta) + x)*myResol) + myOrigin(1);
%             Yocc = ceil((-1*range_set .* sin(scanAngles + theta) + y)*myResol) + myOrigin(2);
             Yocc = ceil((-1 * range_set .* sin(scanAngles + theta) + y)*myResol) + myOrigin(2);
             
             Xocc = min(Xocc, maxX);
             Xocc = max(Xocc, 1);
             Yocc = min(Yocc, maxY);
             Yocc = max(Yocc, 1);
             
             %Dxy = ceil(myResol * [Xocc' Yocc']) + repmat(myOrigin', [measCnt 1]);  % 1081x2
             %iDxy = sub2ind(size(map), Dxy(:,1), Dxy(:,2));
             iDxy = sub2ind(size(map), Yocc, Xocc);
             
             %size(iDxy)
             
             pCorr(i) = sum(map(iDxy) >= thres_occupied)*10 + sum(map(iDxy) <= thres_free)*(-3);
             %pCorr(i) = sum(map(iDxy));
             %size(TMP)
             % Convert to grid index of this occupied cell
             %Dxy(i,:) = TMP';
%              Dxy(1) = min(Dxy(1), maxX);
%              Dxy(1) = max(Dxy(1), 1);
%              Dxy(2) = min(Dxy(2), maxY);
%              Dxy(2) = max(Dxy(2), 1);
%              %iDxy = sub2ind(size(map), Dxy(2), Dxy(1));
%              if map(Dxy) > thres_occupied
%                  pCorr(i) = pCorr(i) + map(Dxy) * 10;
%              elseif map(Dxy) < thres_free  
%                  pCorr(i) = pCorr(i) + map(Dxy) * (-5);
%              end
         %end
     end
 
     %Dxy(:, 1) = min(Dxy(:, 1), maxX);
     %Dxy(:, 1) = max(Dxy(:, 1), 1);
     %Dxy(:, 2) = min(Dxy(:, 2), maxY);
     %Dxy(:, 2) = max(Dxy(:, 2), 1);
     
     %pCorr = pCorr + map(map(Dxy) > thres_occupied)*10;
     %pCorr = pCorr + map(map(Dxy) < thres_free)*(-5);
     if 0
       %pCorr = pCorr - min(pCorr);
       %W = W .* pCorr / sum(pCorr);
       %W = W/sum(W);
       %[maxW maxIdx] = max(W);
     end
     
     pCorr = pCorr - min(pCorr);
     W = W .* pCorr;
     sumW = sum(W);
     W = W/sumW;
     [maxW maxIdx] = max(W);
     
     %   2-2) For each particle, calculate the correlation scores of the particles
     
     
     %   2-3) Update the particle weights         
  
     
     %   2-4) Choose the best particle to update the pose
     myPose(:,j) = P(:,maxIdx);
     
     % 3) Resample if the effective number of particles is smaller than a threshold
     Neffective = power(sum(W),2)/sum(W.^2);
     if Neffective < thres_resample*M
         
         %fprintf("Resample---%d\n", j);
         cumW = cumsum(W);
         Psample = zeros(3, M);
         %Wsample = zeros(1, M);
         for i=1:M
             idx = find(rand < cumW, 1);
             Psample(:,i) = P(:,idx);
          %   Wsample(i) = W(idx);
             %%%%Psample(:,i) = P(:,maxIdx);
             %Wsample(i) = W(maxIdx);

         end

%          Wsample = Wsample - min(Wsample);
%          sumW = sum(Wsample);
%          Wsample = Wsample/sumW;
%          Wsample = W;
     else
         Psample = repmat(myPose(:,j), [1, M]);
         Wsample = W;
     end
          
     
     % 4) Visualize the pose on the map as needed
     if mod(j, 10) == 0   
        figure(1);
        title("Partile Filter: Localizing");
        imagesc(map); colormap('gray'); axis equal; 

        hold on;
        aaX = (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*myResol + myOrigin(1);
        bbY = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*myResol + myOrigin(2);
        plot(aaX, bbY, 'g.');

        hold on;
        plot(ceil(myPose(1,1:j)*myResol) + myOrigin(1), ceil(myPose(2,1:j)*myResol) + myOrigin(2), 'r.-'); 
        %fprintf("(%f, %f)\n", ceil(myPose(1,j)*myResol)+myOrigin(1), ceil(myPose(2,j)*myResol) + myOrigin(2));
        %plot(myPose(1,j)*myResol, myPose(2,j)*myResol, 'b+');
     end

end



% figure(2);
% imagesc(map); colormap('gray'); axis equal;
% hold on;
% plot(myPose(1,:)*param.resol+param.origin(1), myPose(2,:)*param.resol+param.origin(2), 'r.-');
% hold on;




%      for i=1:M     % Particles
%          x = P(1,i);
%          y = P(2,i);
%          theta = P(3,i);
%          
%          xx = ceil(myResol*x) + myOrigin(1);
%          yy = ceil(myResol*y) + myOrigin(2);
%          
%          %for t = 1:measCnt    % Laser measurement
%              Xocc(i,:) = range_set*cos(theta + scanAngles) + x;
%              Yocc(i,:) = -1*range_set*sin(theta + scanAngles) + y;
%              
%              % Convert to grid index of this occupied cell
%              Dxy(i,:) = ceil(myResol * [Xocc(i,:) Yocc(i,:)]') + myOrigin;
%              Dxy(1) = min(Dxy(1), maxX);
%              Dxy(1) = max(Dxy(1), 1);
%              Dxy(2) = min(Dxy(2), maxY);
%              Dxy(2) = max(Dxy(2), 1);
%              %iDxy = sub2ind(size(map), Dxy(2), Dxy(1));
%              if map(Dxy) > thres_occupied
%                  pCorr(i) = pCorr(i) + map(Dxy) * 10;
%              elseif map(Dxy) < thres_free  
%                  pCorr(i) = pCorr(i) + map(Dxy) * (-5);
%              end
%          %end
%      end