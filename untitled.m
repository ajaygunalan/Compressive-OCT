clear all; close all; clc;
folderPath = 'data/salmone/';
filename = 'oct1.jpg';

[depth_points, perpendicular_distances] = depthEstimationFrom2D(folderPath, filename);

% Construct the filename for the final image
[~, name, ext] = fileparts(filename);  % Get the extension dynamically
final_image_filename = fullfile(folderPath, [name 'final' ext]);  % Use the dynamic extension

% Plotting the final_image
figure;
imshow(imread(final_image_filename));
title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image')