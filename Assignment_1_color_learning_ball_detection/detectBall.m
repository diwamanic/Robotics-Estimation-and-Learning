% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%

% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
Samples_struct = load('./yellow_ball_data.mat');
%scatter3(Samples_struct.Samples)

Samples = double(Samples_struct.Samples);
mu = sum(Samples, 1)/size(Samples, 1);

diff = Samples - repmat(mu, size(Samples, 1), 1);

acc_mat = 0;
for i = 1:size(Samples, 1)
    acc_mat = acc_mat + diff(i,:)' * diff(i,:);
end
sigma = acc_mat / size(Samples, 1);

R_min = mu(1) - 1 * sqrt(sigma(1));
R_max = mu(1) + 1 * sqrt(sigma(1));
G_min = mu(2) - 1 * sqrt(sigma(5));
G_max = mu(2) + 1 * sqrt(sigma(5));
B_min = mu(3) - 1 * sqrt(sigma(9));
B_max = mu(3) + 1 * sqrt(sigma(9));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
mask_1 = I(:,:,1) > R_min & I(:,:,1) < R_max;
figure, imshow(mask_1);
mask = I(:,:,1) > R_min & I(:,:,1) < R_max &...
    I(:,:,2) > G_min & I(:,:,2) < G_max & I(:,:,3) > B_min & I(:,:,3) < B_max;
figure, imshow(mask);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
segI = false(size(mask));

CC = bwconncomp(mask);
num_pixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(num_pixels);
segI(CC.PixelIdxList{idx}) = true;
figure,
imshow(segI); hold on;

S = regionprops(CC, 'centroid');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

loc = S(idx).Centroid;
plot(loc(1), loc(2), 'r+');

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
