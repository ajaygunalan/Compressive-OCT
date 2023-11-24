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
%% Upsample and Normalise

% Normalize each matrix to the range [0, 1]
maxTruthData = max(max(TruthData));
maxCompressiveData = max(max(CompressiveData));
TruthData = TruthData ./ maxTruthData; 
Compressive_norm = CompressiveData ./ maxCompressiveData;

% Upsampling rows for CscanCompressionRatio = 0.5
rowUpsampleFactor = 1 / CompressiveMeta.CscanCompressionRatio; 
CompressiveUpsampledRows = zeros(size(CompressiveData, 1) * rowUpsampleFactor, size(CompressiveData, 2));
CompressiveUpsampledRows(1:rowUpsampleFactor:end, :) = CompressiveData;

% Upsampling columns for BscanCompressionRatio = 0.25
colUpsampleFactor = 1 / CompressiveMeta.BscanCompressionRatio; 
CompressiveUpsampled = zeros(size(CompressiveUpsampledRows, 1), size(CompressiveUpsampledRows, 2) * colUpsampleFactor);
CompressiveUpsampled(:, 1:colUpsampleFactor:end) = CompressiveUpsampledRows;

% Normalize the upsampled matrix to the range [0, 1]
CompressiveUpsampled_norm = (CompressiveUpsampled - min(CompressiveUpsampled(:))) / (max(CompressiveUpsampled(:)) - min(CompressiveUpsampled(:)));
%% Get Mask, Y 
A_2dMask = CompressiveUpsampled_norm ~= 0;
% Linearise A
A_1dMask = reshape(A_2dMask, [], 1);
A_LinearIdx = find(A_1dMask == 1);
x = reshape(CompressiveUpsampled_norm, [], 1);
y = x(A_LinearIdx);
%% Reconstruct and Compute the Error 
[Estimation, reconstruction_time] = csAj(A_2dMask, y);
reconstruction_error = RelErr(TruthData, Estimation);

% Scale back the values
TruthData = TruthData .* maxTruthData; 
Estimation = Estimation .* maxCompressiveData;
%% Display & Save Results
% Define the filename for storing error and time
reconstructionResultFilename = [basePath, trialNum, '_ReconstructionResult.csv'];
% Create a table with reconstruction error and time
resultsTable = table(reconstruction_error, reconstruction_time, 'VariableNames', {'ReconstructionError', 'ReconstructionTime'});
% Write the table to a CSV file
writetable(resultsTable, reconstructionResultFilename);
% Display the reconstruction time and error
disp(['Reconstruction Time: ', num2str(reconstruction_time), ' seconds']);
disp(['Reconstruction Error: ', num2str(reconstruction_error), ' %']);

% Display the actual scanning times
disp(['Full Scanning Time for Truth Data: ', num2str(TruthMeta.ActualScanningTimeSec), ' seconds']);
disp(['Compressive Scanning Time for Compressive Data: ', num2str(CompressiveMeta.ActualScanningTimeSec), ' seconds']);


% Save the Normalized TruthData Matrix
fig1 = figure; imagesc(TruthData); axis equal; axis tight; title('TrueData'); colorbar;
saveas(fig1, [basePath, trialNum, 'TrueData.png']);

% Save the Normalized Compressive Matrix
fig2 = figure; imagesc(Compressive_norm); axis equal; axis tight; title('SparseData'); colorbar;
saveas(fig2, [basePath, trialNum, 'SparseData.png']);

% Save the Normalized Upsampled Compressive Matrix
fig3 = figure; imagesc(CompressiveUpsampled_norm); axis equal; axis tight; title('UpsampledSparseData'); colorbar;
saveas(fig3, [basePath, trialNum, 'UpsampledSparseData.png']);

% Save the Estimation
fig4 = figure; imagesc(Estimation); axis equal; axis tight; title('Estimation'); colorbar;
saveas(fig4, [basePath, trialNum, 'Estimation.png']);