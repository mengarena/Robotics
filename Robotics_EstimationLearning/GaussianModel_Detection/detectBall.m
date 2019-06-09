% Robotics: Estimation and Learning 
% Ball detection with Gaussian Model
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

hsv = rgb2hsv(I);
H = hsv(:,:,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = 0.1605; 
sig = 0.0222; 
thre = 0.00007;   % Threshold for probability of being Yellow for a pixel

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
p = [];

[r c] = size(H);
max(H(:))
min(H(:))

for i=1:r
    for j=1:c
       % Gaussian Distribution model for probability of each pixel
       % belonging to Yellow
       p(i,j) = 1/(sig*sqrt(2*pi)) * exp(-1*power(H(i,j)-mu, 2)/(2*sig*sig));
    end
end

%histogram(p)
max(p(:))
min(p(:))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% create new empty binary image
bw = p > thre;

%segI = bw;

bw_biggest = false(size(bw));

CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
%figure,
%imshow(bw_biggest); hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
%plot(loc(1), loc(2),'r+');

segI = bw_biggest;

end
