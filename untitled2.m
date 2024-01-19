clear all; close all; clc;
% Load data
raw = readmatrix("data/getDepthFromSparse3Doct/1/ScanNum_3_Estimation.csv");

% Define common filter parameters
fc = 450; % cutoff frequency
fs = 1000; % sampling frequency
neigbourSz = 5;
sigma = 0.5;

% Butterworth Filter
[b, a] = butter(6, fc/(fs/2)); 
dataOutButter = filter(b, a, raw); 

% Chebyshev Type I Filter
[b, a] = cheby1(6, 10, fc/(fs/2)); 
dataOutCheby = filter(b, a, raw); 

% Elliptic Filter
[b, a] = ellip(6, 3, 50, fc/(fs/2)); 
dataOutEllip = filter(b, a, raw); 

% Custom1
filtered1 = customFilter1(raw, neigbourSz, sigma);

% Custom2
filtered2 = customFilter2(raw, neigbourSz, sigma); 

% Plotting
figure;

subplot(3, 2, 1);
imagesc(raw);
title('Original Data');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;

subplot(3, 2, 2);
imagesc(dataOutButter);
title('Butterworth Filter');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;

subplot(3, 2, 3);
imagesc(dataOutCheby);
title('Chebyshev Type I Filter');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;

subplot(3, 2, 4);
imagesc(dataOutEllip);
title('Elliptic Filter');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;

subplot(3, 2, 5);
imagesc(filtered1);
title('Custom Low-Pass Filter');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;

subplot(3, 2, 6);
imagesc(filtered2);
title('Conservative Smoothing and Gaussian Filter');
axis equal;
colorbar;
hold on;
rectangle('Position', [250, 125, 100, 50], 'EdgeColor', 'r', 'LineWidth', 2);
hold off;


% Custom Low-Pass Filter Function
function [nImg, mask] = customFilter1(img, neigbourSz, sigma)
    r = sigma;
    F = fftshift(fft2(img));
    mask = fspecial('gaussian', [neigbourSz neigbourSz], r);
    M = fft2(mask, size(F,1), size(F,2));
    Filtered = M.*F;
    nImg = real(ifft2(ifftshift(Filtered)));
end

% Conservative Smoothing and Gaussian Filter
function filter2 = customFilter2(img, neighborhoodSize, sigma)
    filter1 = conservativeSmoothingFilter(img, neighborhoodSize);
    filter2 = imgaussfilt(filter1, sigma);
end