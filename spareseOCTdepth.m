clear all; clc; close all;

% Specify the file path
filenameF = 'data\getDepthFromSparse3Doct\surfaceFull.csv';
filenameC = 'data\getDepthFromSparse3Doct\surfaceCompressive.csv';
filenameCmeta = 'data\getDepthFromSparse3Doct\surfaceCompressive_meta.csv';

% Read the matrix from the CSV file
Full = readmatrix(filenameF);
Compressive = readmatrix(filenameC);
CompressiveMeta = readmatrix(filenameCmeta);

% Assign variables
BScansPerVolume = CompressiveMeta(1);
AScansPerBScan = CompressiveMeta(2);
ActualScanningTimeSec = CompressiveMeta(3);
FullBScansPerVolume = CompressiveMeta(4);
FullAScansPerBScan = CompressiveMeta(5);
BscanCompressionRatio = CompressiveMeta(6);
AscanCompressionRatio = CompressiveMeta(7);
CompressiveBScansPerVolume = CompressiveMeta(8);
CompressiveAScansPerBScan = CompressiveMeta(9);
LengthOfBScan = CompressiveMeta(10);
WidthOfVolume = CompressiveMeta(11);
NumOfLostBScan = CompressiveMeta(12);
ExpectedAcquisitionTimeSec = CompressiveMeta(13);

% Remove the last column from each matrix
Full(:, end) = [];
Compressive(:, end) = [];

% Normalize each matrix to the range [0, 1]
Full_norm = (Full - min(Full(:))) / (max(Full(:)) - min(Full(:)));
Compressive_norm = (Compressive - min(Compressive(:))) / (max(Compressive(:)) - min(Compressive(:)));


% Upsampling rows for BscanCompressionRatio = 0.5
rowUpsampleFactor = 1 / BscanCompressionRatio; 
CompressiveUpsampledRows = zeros(size(Compressive, 1) * rowUpsampleFactor, size(Compressive, 2));
CompressiveUpsampledRows(1:rowUpsampleFactor:end, :) = Compressive;

% Upsampling columns for AscanCompressionRatio = 0.25
colUpsampleFactor = 1 / AscanCompressionRatio; 
CompressiveUpsampled = zeros(size(CompressiveUpsampledRows, 1), size(CompressiveUpsampledRows, 2) * colUpsampleFactor);
CompressiveUpsampled(:, 1:colUpsampleFactor:end) = CompressiveUpsampledRows;

% Normalize the upsampled matrix to the range [0, 1]
CompressiveUpsampled_norm = (CompressiveUpsampled - min(CompressiveUpsampled(:))) / (max(CompressiveUpsampled(:)) - min(CompressiveUpsampled(:)));

%%
% Display the normalized Full Matrix
figure;
imagesc(Full_norm);
axis equal;
axis tight;
title('Normalized Full Matrix');
colorbar;

% Display the normalized Compressive Matrix
figure;
imagesc(Compressive_norm);
axis equal;
axis tight;
title('Normalized Compressive Matrix');
colorbar;

% Display the normalized Upsampled Compressive Matrix
figure;
imagesc(CompressiveUpsampled_norm);
axis equal;
axis tight;
title('Normalized Upsampled Compressive Matrix');
colorbar;
%%


