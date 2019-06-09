% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples_HSV = [];
 for k=1:15
     % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    hsv = rgb2hsv(I);
    
    % You may consider other color space than RGB
    H = hsv(:,:,1);
    S = hsv(:,:,2); 
    V = hsv(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(hsv); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    H = H(sample_ind);
    S = S(sample_ind);
    V = V(sample_ind);
    
    Samples_HSV = [Samples_HSV; [H S V]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
 end

% visualize the sample distribution
figure, 
scatter3(Samples_HSV(:,1),Samples_HSV(:,2),Samples_HSV(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Hue');
ylabel('Saturation');
zlabel('Value');

figure,
histogram(Samples_HSV(:,1));
title('Hue Distribubtion');
xlabel('Hue');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

muH = mean(Samples_HSV(:,1));
sigmaH = std(Samples_HSV(:,1));

