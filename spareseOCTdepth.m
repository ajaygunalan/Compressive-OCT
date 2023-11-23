clear all; clc; close all;
%% Perform Imagaing
% Prompt the user for the trial number
trialNum = input('Enter the trial number: ', 's');
commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum];
[status, cmdout] = system(commandStr);
% Check the status and display the output or error message
if status == 0
    disp('Command executed successfully');
    disp(cmdout);
else
    disp('Error in executing command');
    disp(cmdout);
end
%% Read the Data
trialNum = '1';
% Base path
basePath = 'data\getDepthFromSparse3Doct\';

% File paths
TruthPath = [basePath, trialNum, 'surfaceTruth.csv'];
TruthMetaPath = [basePath, trialNum, 'surfaceTruth_meta.csv'];
CompressivePath = [basePath, trialNum, 'surfaceCompressive.csv'];
CompressiveMetaPath = [basePath, trialNum, 'surfaceCompressive_meta.csv'];

% Read the matrix from the CSV file
TruthData = readmatrix(TruthPath);
TruthMetaData = readmatrix(TruthMetaPath);
CompressiveData = readmatrix(CompressivePath);
CompressiveMetaData = readmatrix(CompressiveMetaPath);

% Creating metadata structures
TruthMeta = struct();
CompressiveMeta = struct();

% Assigning values to TruthMeta structure
TruthMeta.BScansPerVolume = TruthMetaData(1);
TruthMeta.AScansPerBScan = TruthMetaData(2);
TruthMeta.BscanCompressionRatio = TruthMetaData(3);
TruthMeta.CscanCompressionRatio = TruthMetaData(4);
TruthMeta.ActualScanningTimeSec = TruthMetaData(5);
TruthMeta.LengthOfBScan = TruthMetaData(6);
TruthMeta.WidthOfVolume = TruthMetaData(7);
TruthMeta.NumOfLostBScan = TruthMetaData(8);
TruthMeta.ExpectedAcquisitionTimeSec = TruthMetaData(9);

% Assigning values to CompressiveMeta structure
CompressiveMeta.BScansPerVolume = CompressiveMetaData(1);
CompressiveMeta.AScansPerBScan = CompressiveMetaData(2);
CompressiveMeta.BscanCompressionRatio = CompressiveMetaData(3);
CompressiveMeta.CscanCompressionRatio = CompressiveMetaData(4);
CompressiveMeta.ActualScanningTimeSec = CompressiveMetaData(5);
CompressiveMeta.LengthOfBScan = CompressiveMetaData(6);
CompressiveMeta.WidthOfVolume = CompressiveMetaData(7);
CompressiveMeta.NumOfLostBScan = CompressiveMetaData(8);
CompressiveMeta.ExpectedAcquisitionTimeSec = CompressiveMetaData(9);

% Remove the last column from each data matrix
TruthData(:, end) = [];
CompressiveData(:, end) = [];
%%

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