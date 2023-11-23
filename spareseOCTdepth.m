clear all; clc; close all;
%%

system('"\\bin\\x64\\Debug\\OCTImageCapture.exe"')


% Specify the file path
filenameT = 'data\getDepthFromSparse3Doct\surfaceTruth.csv';
filenameC = 'data\getDepthFromSparse3Doct\surfaceCompressive.csv';
filenameCmeta = 'data\getDepthFromSparse3Doct\surfaceCompressive_meta.csv';

% Read the matrix from the CSV file
Truth = readmatrix(filenameT);
Compressive = readmatrix(filenameC);
CompressiveMeta = readmatrix(filenameCmeta);

% Assign variables
BScansPerVolume = CompressiveMeta(1);
AScansPerBScan = CompressiveMeta(2);
ActualScanningTimeSec = CompressiveMeta(3);
TruthBScansPerVolume = CompressiveMeta(4);
TruthAScansPerBScan = CompressiveMeta(5);
BscanCompressionRatio = CompressiveMeta(6);
CscanCompressionRatio = CompressiveMeta(7);
CompressiveBScansPerVolume = CompressiveMeta(8);
CompressiveAScansPerBScan = CompressiveMeta(9);
LengthOfBScan = CompressiveMeta(10);
WidthOfVolume = CompressiveMeta(11);
NumOfLostBScan = CompressiveMeta(12);
ExpectedAcquisitionTimeSec = CompressiveMeta(13);

% Remove the last column from each matrix
Truth(:, end) = [];
Compressive(:, end) = [];

% Normalize each matrix to the range [0, 1]
Truth = (Truth - min(Truth(:))) / (max(Truth(:)) - min(Truth(:)));
Compressive_norm = (Compressive - min(Compressive(:))) / (max(Compressive(:)) - min(Compressive(:)));

% Upsampling columns for BscanCompressionRatio = 0.25
colUpsampleFactor = 1 / BscanCompressionRatio; 
CompressiveUpsampled = zeros(size(CompressiveUpsampledRows, 1), size(CompressiveUpsampledRows, 2) * colUpsampleFactor);
CompressiveUpsampled(:, 1:colUpsampleFactor:end) = CompressiveUpsampledRows;

% Upsampling rows for CscanCompressionRatio = 0.5
rowUpsampleFactor = 1 / CscanCompressionRatio; 
CompressiveUpsampledRows = zeros(size(Compressive, 1) * rowUpsampleFactor, size(Compressive, 2));
CompressiveUpsampledRows(1:rowUpsampleFactor:end, :) = Compressive;

% Normalize the upsampled matrix to the range [0, 1]
CompressiveUpsampled_norm = (CompressiveUpsampled - min(CompressiveUpsampled(:))) / (max(CompressiveUpsampled(:)) - min(CompressiveUpsampled(:)));
%%
A_2dMask = CompressiveUpsampled_norm ~= 0;
% Linearise A
A_1dMask = reshape(A_2dMask, [], 1);
A_LinearIdx = find(A_1dMask == 1);
x = reshape(CompressiveUpsampled_norm, [], 1);
y = x(A_LinearIdx);

[Estimation, reconstruction_time] = csAj(A_2dMask, y);
reconstruction_error = RelErr(Truth, Estimation);
%% Display

% Display the normalized Truth Matrix
figure;
imagesc(Truth);
axis equal;
axis tight;
title('Normalized Truth Matrix');
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

% Display the normalized Upsampled Compressive Matrix
figure;
imagesc(Estimation);
axis equal;
axis tight;
title('Estimation');
colorbar;