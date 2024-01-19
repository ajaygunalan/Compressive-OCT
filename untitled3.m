clear all;
close all;
clc;

% Load data
raw = readmatrix("data/getDepthFromSparse3Doct/1/ScanNum_3_Estimation.csv");

% Define the threshold value
threshold = 0.3; % You can adjust this threshold value as needed

% Plotting
figure;

subplot(1,1, 1);
imagesc(raw);
title('Original Data');
axis equal;
colorbar;

% Apply thresholding and highlight the ROI in red
hold on;
ROI = raw < threshold; % Create a binary mask for values less than the threshold
red_mask = cat(3, ROI, zeros(size(ROI)), zeros(size(ROI))); % Create an RGB mask for red color
h = imshow(red_mask, 'InitialMagnification', 'fit');
set(h, 'AlphaData', ROI); % Use the binary mask as alpha data for transparency
hold off;

% Define the ROI rectangle (adjust these values as needed)
roi_x = 250;
roi_y = 125;
roi_width = 100;
roi_height = 50;
rectangle('Position', [roi_x, roi_y, roi_width, roi_height], 'EdgeColor', 'r', 'LineWidth', 2);
