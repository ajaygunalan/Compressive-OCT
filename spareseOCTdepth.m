clear all; clc; close all;
% %% Perform Imagaing
% % Prompt the user for the trial number
% trialNum = input('Enter the trial number: ', 's');
% commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum];
% [status, cmdout] = system(commandStr);
% % Check the status and display the output or error message
% % Check the status and display the output or error message
% if status == 0
%     disp(['Command executed successfully: ', cmdout]);
% else
%     disp(['Error in executing command: ', cmdout]);
% end
%% 
trialNum = '5';
% Intialize Variable
defaultVal = -143;
matrixSize = [10, 10];
ReconstructionError = defaultVal * ones(matrixSize);
ReconstructionTime = defaultVal * ones(matrixSize);
ScanTime = defaultVal * ones(matrixSize);
NetReductionFactor = defaultVal * ones(matrixSize);

TruthDataSize = [300, 300];
TruthData = defaultVal * ones(TruthDataSize);
maxTruthData = 0;
TruthMeta = struct();

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

% Initialize counter
count = 1;
% Define the list of tuples (pairs) for BscanCR and CscanCR
compressionPairs = {
    [1.0, 1.0], [0.5, 0.5], ...
                [1.0, 0.9], [1.0, 0.8], [1.0, 0.5], ...
    [0.9, 1.0], [0.9, 0.9], [0.9, 0.8], [0.9, 0.5], ...
    [0.8, 1.0], [0.8, 0.9], [0.8, 0.8], [0.8, 0.5], ...
    [0.5, 1.0], [0.5, 0.9], [0.5, 0.8]
};

% Initialize count
count = 1;
% Iterate over each pair
for idx = 1:length(compressionPairs)
    pair = compressionPairs{idx};
    BscanCR = pair(1);
    CscanCR = pair(2);

    % Construct filename
    fileName = sprintf('ScanNum_%d_CR_BScan_%0.2f_CScan_%0.2f', count, BscanCR, CscanCR);
    surfaceFileName = fullfile(folderLocation, [fileName, '_surface.csv']);
    metaFileName = fullfile(folderLocation, [fileName, '_meta.csv']);
    prefix = sprintf('ScanNum_%d_', count);

    % Check if BscanCR and CscanCR are both 1.0
    if BscanCR == 1.0 && CscanCR == 1.0
        TruthData = readmatrix(surfaceFileName);
        TruthMetaData = readmatrix(metaFileName);
        TruthData(:, end) = [];

        % Assigning values to TruthMeta structure
        TruthMeta.BScansPerVolume = TruthMetaData(1, 2);
        TruthMeta.AScansPerBScan = TruthMetaData(2, 2);
        TruthMeta.ActualScanningTimeSec = TruthMetaData(3, 2);
        TruthMeta.ExpectedAcquisitionTimeSec = TruthMetaData(4, 2);
        TruthMeta.NumOfLostBScan = TruthMetaData(5, 2);
        ScanTime(BscanCR*10, CscanCR*10) = TruthMeta.ActualScanningTimeSec;


        % Create the figure and plot the data
        minData = min(TruthData(:)); 
        maxData = max(TruthData(:));
        fig1 = figure('Visible', 'off'); 
        imagesc(TruthData); axis equal; axis tight; 
        % Create the colorbar and set its limits
        cb = colorbar; 
        set(cb, 'Limits', [minData, maxData]);
        % Define new ticks (for example, 5 evenly spaced ticks)
        T = linspace(minData, maxData, 5); 
        % Set these new ticks on the colorbar
        set(cb, 'Ticks', T); 
        % Format each tick label to two decimal places
        TL = arrayfun(@(x) sprintf('%.2f', x), T, 'UniformOutput', false); 
        % Assign these formatted labels to the colorbar
        set(cb, 'TickLabels', TL);

        % Save the figure as a MATLAB figure file
        imageFilename = [prefix, 'TrueDataNoisy.svg'];
        fullImagePath = fullfile(folderLocation, imageFilename); 
        saveas(fig1, fullImagePath, 'svg');
         % Save Compressive_norm data as a text file
        textFilename = [prefix, 'TrueDataNoisy.csv'];
        fullTextPath = fullfile(folderLocation, textFilename);
        writematrix(TruthData, fullTextPath);

        % Define the nfilter
        neighborhoodSize = 5; 
        % Apply the filter to remove outlier
        TruthData = conservativeSmoothingFilter(TruthData, neighborhoodSize);
        % Define the standard deviation for the Gaussian filter
        sigma = 0.6; 
        % Apply the Gaussian filter to the TruthData
        TruthData = imgaussfilt(TruthData, sigma);
        
%         % Display the original TruthData
%         figure;
%         subplot(1, 2, 1);
%         imagesc(TruthData);
%         title('Original TruthData');
%         colorbar;
%         axis equal;
%         axis tight;
%         
%         % Display the filtered data
%         subplot(1, 2, 2);
%         imagesc(filteredData);
%         title('Filtered Data with Median Filter');
%         colorbar;
%         axis equal;
%         axis tight;

        % Create the figure and plot the data
        minData = min(TruthData(:)); 
        maxData = max(TruthData(:));
        fig1 = figure('Visible', 'off'); 
        imagesc(TruthData); axis equal; axis tight; 
        % Create the colorbar and set its limits
        cb = colorbar; 
        set(cb, 'Limits', [minData, maxData]);
        % Define new ticks (for example, 5 evenly spaced ticks)
        T = linspace(minData, maxData, 5); 
        % Set these new ticks on the colorbar
        set(cb, 'Ticks', T); 
        % Format each tick label to two decimal places
        TL = arrayfun(@(x) sprintf('%.2f', x), T, 'UniformOutput', false); 
        % Assign these formatted labels to the colorbar
        set(cb, 'TickLabels', TL);

        % Save the figure as a MATLAB figure file
        imageFilename = [prefix, 'TrueData.svg'];
        fullImagePath = fullfile(folderLocation, imageFilename); 
        saveas(fig1, fullImagePath, 'svg');
         % Save Compressive_norm data as a text file
        textFilename = [prefix, 'TrueData.csv'];
        fullTextPath = fullfile(folderLocation, textFilename);
        writematrix(TruthData, fullTextPath);

        % Normalize each matrix to the range [0, 1]
        maxTruthData = max(max(TruthData));
        TruthData = TruthData ./ maxTruthData;
    else
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
        %% Reconstruct and Compute the Error 
        [Estimation, reconstruction_time] = csAj(A_2dMask, y);
        reconstruction_error = RelErr(TruthData, Estimation);
        value = (TruthMeta.ActualScanningTimeSec - (reconstruction_time + CompressiveMeta.ActualScanningTimeSec))/TruthMeta.ActualScanningTimeSec;
        
        NetReductionFactor(round(BscanCR*10), round(CscanCR*10)) = value;
        ReconstructionTime(round(BscanCR*10), round(CscanCR*10)) = reconstruction_time;
        ScanTime(round(BscanCR*10), round(CscanCR*10)) = CompressiveMeta.ActualScanningTimeSec;
        ReconstructionError(round(BscanCR*10), round(CscanCR*10)) = reconstruction_error;

        % Scale back the values
        TruthData = TruthData * maxTruthData; 
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
        imagesc(CompressiveNorm); axis equal; axis tight; 
        % Create the colorbar and set its limits
        minData = min(CompressiveNorm(:)); 
        maxData = max(CompressiveNorm(:));
        cb = colorbar; 
        set(cb, 'Limits', [minData, maxData]);
        % Define new ticks (for example, 5 evenly spaced ticks)
        T = linspace(minData, maxData, 5); 
        % Set these new ticks on the colorbar
        set(cb, 'Ticks', T); 
        % Format each tick label to two decimal places
        TL = arrayfun(@(x) sprintf('%.2f', x), T, 'UniformOutput', false); 
        % Assign these formatted labels to the colorbar
        set(cb, 'TickLabels', TL);
        % Save the figure as a MATLAB figure file
        imageFilename = [prefix, 'SparseData.svg'];
        fullImagePath = fullfile(folderLocation, imageFilename); 
        saveas(fig1, fullImagePath, 'svg');

        % Save CompressiveUpsampled data as a figure with detailed colorbar settings
        fig2 = figure('Visible', 'off'); 
        imagesc(CompressiveUpsampled); axis equal; axis tight; 
        % Create the colorbar and set its limits
        minDataUpsampled = min(CompressiveUpsampled(:)); 
        maxDataUpsampled = max(CompressiveUpsampled(:));
        cb2 = colorbar; 
        set(cb2, 'Limits', [minDataUpsampled, maxDataUpsampled]);
        % Define new ticks for CompressiveUpsampled
        T2 = linspace(minDataUpsampled, maxDataUpsampled, 5); 
        set(cb2, 'Ticks', T2); 
        TL2 = arrayfun(@(x) sprintf('%.2f', x), T2, 'UniformOutput', false); 
        set(cb2, 'TickLabels', TL2);
        % Save the figure as a MATLAB figure file
        imageFilename2 = [prefix, 'UpsampledSparseData.svg'];
        fullImagePath2 = fullfile(folderLocation, imageFilename2); 
        saveas(fig2, fullImagePath2, 'svg');

        % Save Estimation data as a figure with detailed colorbar settings
        fig3 = figure('Visible', 'off'); 
        imagesc(Estimation); axis equal; axis tight; 
        % Create the colorbar and set its limits
        minDataEstimation = min(Estimation(:)); 
        maxDataEstimation = max(Estimation(:));
        cb3 = colorbar; 
        set(cb3, 'Limits', [minDataEstimation, maxDataEstimation]);
        % Define new ticks for Estimation
        T3 = linspace(minDataEstimation, maxDataEstimation, 5); 
        set(cb3, 'Ticks', T3); 
        TL3 = arrayfun(@(x) sprintf('%.2f', x), T3, 'UniformOutput', false); 
        set(cb3, 'TickLabels', TL3);
        % Save the figure as a MATLAB figure file
        imageFilename3 = [prefix, 'Estimation.svg'];
        fullImagePath3 = fullfile(folderLocation, imageFilename3); 
        saveas(fig3, fullImagePath3, 'svg');
    end
    count = count + 1;
end
saveFileName = fullfile(folderLocation, 'Results.mat');
save(saveFileName, 'NetReductionFactor', 'ReconstructionTime', 'ScanTime', 'ReconstructionError');