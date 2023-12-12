clear all; clc; close all;
%%
% Set base folder
baseFolder = 'C:\Ajay_OCT\OCT-Guided-AutoCALM\data\getDepthFromSparse3Doct\';

% Number of trials
numTrials = 2;

% Initialize arrays to store results
allNetReductionFactors = zeros(10, 10, numTrials);
allReconstructionTimes = zeros(10, 10, numTrials);
allScanTimes = zeros(10, 10, numTrials);
allReconstructionErrors = zeros(10, 10, numTrials);

% Iterate through each trial
for trialNum = 1:numTrials
    % Construct folder location for each trial
    folderLocation = fullfile(baseFolder, num2str(trialNum));

    % Construct file name for the results file
    saveFileName = fullfile(folderLocation, 'Results.mat');

    % Load the data if the file exists
    if exist(saveFileName, 'file')
        load(saveFileName, 'NetReductionFactor', 'ReconstructionTime', 'ScanTime', 'ReconstructionError');
        allNetReductionFactors(:, :, trialNum) = NetReductionFactor;
        allReconstructionTimes(:, :, trialNum) = ReconstructionTime;
        allScanTimes(:, :, trialNum) = ScanTime;
        allReconstructionErrors(:, :, trialNum) = ReconstructionError;
    else
        fprintf('Warning: Results.mat not found for trial number %d\n', trialNum);
    end
end

% Calculate the mean and standard deviation for each metric
meanNetReductionFactors = mean(allNetReductionFactors, 3);
stdDevNetReductionFactors = std(allNetReductionFactors, 0, 3);

meanReconstructionTimes = mean(allReconstructionTimes, 3);
stdDevReconstructionTimes = std(allReconstructionTimes, 0, 3);

meanScanTimes = mean(allScanTimes, 3);
stdDevScanTimes = std(allScanTimes, 0, 3);

meanReconstructionErrors = mean(allReconstructionErrors, 3);
stdDevReconstructionErrors = std(allReconstructionErrors, 0, 3);