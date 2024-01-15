clear all; clc; close all;
%% Perform Imagaing
% Prompt the user for the trial number
trialNum = input('Enter the trial number: ', 's');
commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum];
[status, cmdout] = system(commandStr);
% Check the status and display the output or error message
if status == 0
    disp(['Command executed successfully: ', cmdout]);
else
    disp(['Error in executing command: ', cmdout]);
end
%% 
trialNum = '0';
keys = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
values = {
    [9, 1],  
    [4, 1],  
    [7, 3],  
    [3, 2],  
    [1, 1],     
    [2, 3],  
    [3, 7],  
    [1, 4],  
    [1, 9]   
};              
upSamplingParam = containers.Map(keys, values);
             
%  Set base folder and trial number
baseFolder = 'C:\Ajay_OCT\OCT-Guided-AutoCALM\data\getDepthFromSparse3Doct\';
folderLocation = fullfile(baseFolder, trialNum);
compressionPairs = {[0.5, 0.5]};
pair = compressionPairs{1};
BscanCR = pair(1);
CscanCR = pair(2);

% Construct filename
count = 1;
fileName = sprintf('ScanNum_%d_CR_BScan_%0.2f_CScan_%0.2f', count, BscanCR, CscanCR);
surfaceFileName = fullfile(folderLocation, [fileName, '_surface.csv']);
metaFileName = fullfile(folderLocation, [fileName, '_meta.csv']);
prefix = sprintf('ScanNum_%d_', count);


CompressiveData = readmatrix(surfaceFileName);
CompressiveMetaData = readmatrix(metaFileName);
CompressiveMeta = struct();

% Assigning values to CompressiveMeta structure
CompressiveMeta.BScansPerVolume = CompressiveMetaData(1, 2);  
CompressiveMeta.AScansPerBScan = CompressiveMetaData(2, 2);   
CompressiveMeta.ActualScanningTimeSec = CompressiveMetaData(3, 2); 
CompressiveMeta.ExpectedAcquisitionTimeSec = CompressiveMetaData(4, 2); 
CompressiveMeta.NumOfLostBScan = CompressiveMetaData(5, 2);  

% Remove the last column from each data matrix
CompressiveData(:, end) = [];
% Normalize it
maxCompressiveData = max(max(CompressiveData));
CompressiveNorm = CompressiveData ./ maxCompressiveData;
CompressiveUpsampled = CompressiveNorm;

% Normalize each matrix to the range [0, 1]
fixedMinData = min(CompressiveData(:));
fixedMaxData = max(CompressiveData(:));
maxCompressiveData = max(max(CompressiveData));
CompressiveData = CompressiveData ./ maxCompressiveData;


% Upsampling Columns
if BscanCR ~= 1.0
    val = upSamplingParam(round(BscanCR, 1));
    intervalSize = val(1);
    padSize = val(2);
    tempMatrix = [];
            
    for startIdx = 1:intervalSize:size(CompressiveUpsampled, 2)
        % Intervals 
        endIdx = startIdx + intervalSize - 1;
        selectedColumns = CompressiveUpsampled(:, startIdx:endIdx);
        tempMatrix = [tempMatrix, selectedColumns];
        % Pad Zeros
        zeroColumns = zeros(size(CompressiveUpsampled, 1), padSize);
        tempMatrix = [tempMatrix, zeroColumns];
    end
    CompressiveUpsampled = tempMatrix;
end

% Upsampling Rows
if CscanCR ~= 1.0
    val = upSamplingParam(round(CscanCR, 1));
    intervalSize = val(1);
    padSize = val(2);
    tempMatrix = [];
    for startIdx = 1:intervalSize:size(CompressiveUpsampled, 1)  
        % Intervals
        endIdx = startIdx + intervalSize - 1;
        selectedRows = CompressiveUpsampled(startIdx:endIdx, :);
        tempMatrix = [tempMatrix; selectedRows];  
        % Pad Zeros
        zeroRows = zeros(padSize, size(CompressiveUpsampled, 2));  
        tempMatrix = [tempMatrix; zeroRows]; 
    end
    CompressiveUpsampled = tempMatrix;
end
                
% Get Mask, Y 
A_2dMask = CompressiveUpsampled ~= 0;
% Linearise A
A_1dMask = reshape(A_2dMask, [], 1);
A_LinearIdx = find(A_1dMask == 1);
% Get x and y
x = reshape(CompressiveUpsampled, [], 1);
y = x(A_LinearIdx);

% Reconstruct and Compute the Error 
[Estimation, reconstruction_time] = csAj(A_2dMask, y);

% Scale back the values
CompressiveNorm = CompressiveNorm * maxCompressiveData;
CompressiveUpsampled = CompressiveUpsampled * maxCompressiveData;
Estimation = Estimation * maxCompressiveData;

% Save Compressive_norm data as a text file
textFilename = [prefix, 'SparseData.csv'];
fullTextPath = fullfile(folderLocation, textFilename);
writematrix(CompressiveNorm, fullTextPath);

% Save CompressiveUpsampled data as a text file
textFilename2 = [prefix, 'UpsampledSparseData.csv'];
fullTextPath2 = fullfile(folderLocation, textFilename2);
writematrix(CompressiveUpsampled, fullTextPath2);

% Save Estimation data as a text file
textFilename3 = [prefix, 'Estimation.csv'];
fullTextPath3 = fullfile(folderLocation, textFilename3);
writematrix(Estimation, fullTextPath3);


% Save Compressive_norm data as a figure
fig1 = figure('Visible', 'off'); 
imagesc(CompressiveNorm); 
pbaspect([1 1 1])
axis tight;
minData = fixedMinData; 
maxData = fixedMaxData;
cb = colorbar; 
set(cb, 'Limits', [minData, maxData]);
T = linspace(minData, maxData, 5); 
set(cb, 'Ticks', T); 
TL = arrayfun(@(x) sprintf('%.2f', x), T, 'UniformOutput', false); 
set(cb, 'TickLabels', TL);
ylabel(cb, 'mm');
imageFilename = [prefix, 'SparseData.png'];
fullImagePath = fullfile(folderLocation, imageFilename); 
saveas(fig1, fullImagePath, 'png');

% Save CompressiveUpsampled data as a figure with detailed colorbar settings
fig2 = figure('Visible', 'off'); 
imagesc(CompressiveUpsampled);
pbaspect([1 1 1])
axis tight;
minDataUpsampled = fixedMinData;
maxDataUpsampled = fixedMaxData;
cb2 = colorbar; 
set(cb2, 'Limits', [minDataUpsampled, maxDataUpsampled]);
T2 = linspace(minDataUpsampled, maxDataUpsampled, 5); 
set(cb2, 'Ticks', T2); 
ylabel(cb2, 'mm');
TL2 = arrayfun(@(x) sprintf('%.2f', x), T2, 'UniformOutput', false); 
set(cb2, 'TickLabels', TL2);
imageFilename2 = [prefix, 'UpsampledSparseData.png'];
fullImagePath2 = fullfile(folderLocation, imageFilename2); 
saveas(fig2, fullImagePath2, 'png');

% Save Estimation data as a figure with detailed colorbar settings
fig3 = figure('Visible', 'off'); 
imagesc(Estimation);
pbaspect([1 1 1])
axis tight;
minDataEstimation = fixedMinData;
maxDataEstimation = fixedMaxData;
cb3 = colorbar; 
set(cb3, 'Limits', [minDataEstimation, maxDataEstimation]);
T3 = linspace(minDataEstimation, maxDataEstimation, 5); 
set(cb3, 'Ticks', T3); 
ylabel(cb3, 'mm');
TL3 = arrayfun(@(x) sprintf('%.2f', x), T3, 'UniformOutput', false); 
set(cb3, 'TickLabels', TL3);
% Save the figure as a MATLAB figure file
imageFilename3 = [prefix, 'Estimation.png'];
fullImagePath3 = fullfile(folderLocation, imageFilename3); 
saveas(fig3, fullImagePath3, 'png');