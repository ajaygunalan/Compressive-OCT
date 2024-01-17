clear all; close all; clc;
% Load data
raw = readmatrix("data/getDepthFromSparse3Doct/1/ScanNum_1_Estimation.csv");

% Define common filter parameters
fc = 300; % cutoff frequency
fs = 1000; % sampling frequency

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
filtered1 = customFilter1(raw, 25);

% Custom2
filtered2 = customFilter2(raw, 25, 0.6); 

% Plotting
figure;

subplot(3, 2, 1);
imagesc (raw);
title('Original Data');
colorbar;

subplot(3, 2, 2);
imagesc(dataOutButter);
title('Butterworth Filter');
colorbar;

subplot(3, 2, 3);
imagesc(dataOutCheby);
title('Chebyshev Type I Filter');
colorbar;

subplot(3, 2, 4);
imagesc(dataOutEllip);
title('Elliptic Filter');
colorbar;

subplot(3, 2, 5);
imagesc(filtered1);
title('Custom Low-Pass Filter');
colorbar;

subplot(3, 2, 6);
imagesc(filtered2);
title('Conservative Smoothing and Gaussian Filter');
colorbar;


% Custom Low-Pass Filter Function
function [nImg, mask] = customFilter1(img, r)
    F = fftshift(fft2(img));
    mask = fspecial('gaussian', [3 3], r);
    M = fft2(mask, size(F,1), size(F,2));
    Filtered = M.*F;
    nImg = real(ifft2(ifftshift(Filtered)));
end

% Conservative Smoothing and Gaussian Filter
function filter2 = customFilter2(img, neighborhoodSize, sigma)
    filter1 = conservativeSmoothingFilter(img, neighborhoodSize);
    filter2 = imgaussfilt(filter1, sigma);
end