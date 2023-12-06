clear all; clc; close all;
% %% Perform Imagaing
% % Prompt the user for the trial number
% trialNum = input('Enter the trial number: ', 's');
% commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum];
% [status, cmdout] = system(commandStr);
% % Check the status and display the output or error message
% if status == 0
%     disp('Command executed successfully');
%     disp(cmdout);
% else
%     disp('Error in executing command');
%     disp(cmdout);
% end
%% 
trialNum = '0';
% Intialize Variable
ReconstructionError = -1 * ones(10, 10);
ReconstructionTime = -1 * ones(10, 10);
ScanTime = -1 * ones(10, 10);
NetReductionFactor = -1 * ones(10, 10);

TruthData = -1 *ones(100, 256);
TruthMetaData = [];
maxTruthData= 0;
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

% Parameters for file naming
NumAScansPerBScanReference = 256;
NumBScansPerVolumeReference = 100;

% Initialize counter
count = 1;
% Loop through the range of compression ratios
for BscanCR = 1.0:-0.1:0.1
    for CscanCR = 1.0:-0.1:0.1
        numAScansPerBScan = round(NumAScansPerBScanReference * BscanCR);
        numBScansPerVolume = round(NumBScansPerVolumeReference * CscanCR);

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
            imageFilename = [prefix, 'TrueData.svg'];
            fullImagePath = fullfile(folderLocation, imageFilename); 
            saveas(fig1, fullImagePath, 'svg');

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

            % Up Sampling ColumnS
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

            % Up Sampling Rows
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
end

saveFileName = fullfile(folderLocation, 'Results.mat');
save(saveFileName, 'NetReductionFactor', 'ReconstructionTime', 'ScanTime', 'ReconstructionError');
%%
% %% Read the Data
% % Base path
% basePath = ['data\getDepthFromSparse3Doct\', trialNum, '\'];
% % File paths
% TruthPath = [basePath, 'surfaceTruth.csv'];
% TruthMetaPath = [basePath, 'surfaceTruth_meta.csv'];
% CompressivePath = [basePath, 'surfaceCompressive.csv'];
% CompressiveMetaPath = [basePath, 'surfaceCompressive_meta.csv'];
% 
% % Read the matrix from the CSV file
% TruthData = readmatrix(TruthPath);
% TruthMetaData = readmatrix(TruthMetaPath);
% CompressiveData = readmatrix(CompressivePath);
% CompressiveMetaData = readmatrix(CompressiveMetaPath);
% 
% % Creating metadata structures
% TruthMeta = struct();
% CompressiveMeta = struct();
% 
% % Assigning values to TruthMeta structure
% TruthMeta.BScansPerVolume = TruthMetaData(1);
% TruthMeta.AScansPerBScan = TruthMetaData(2);
% TruthMeta.BscanCompressionRatio = TruthMetaData(3);
% TruthMeta.CscanCompressionRatio = TruthMetaData(4);
% TruthMeta.ActualScanningTimeSec = TruthMetaData(5);
% TruthMeta.LengthOfBScan = TruthMetaData(6);
% TruthMeta.WidthOfVolume = TruthMetaData(7);
% TruthMeta.NumOfLostBScan = TruthMetaData(8);
% TruthMeta.ExpectedAcquisitionTimeSec = TruthMetaData(9);
% 
% % Assigning values to CompressiveMeta structure
% CompressiveMeta.BScansPerVolume = CompressiveMetaData(1);
% CompressiveMeta.AScansPerBScan = CompressiveMetaData(2);
% CompressiveMeta.BscanCompressionRatio = CompressiveMetaData(3);
% CompressiveMeta.CscanCompressionRatio = CompressiveMetaData(4);
% CompressiveMeta.ActualScanningTimeSec = CompressiveMetaData(5);
% CompressiveMeta.ExpectedAcquisitionTimeSec = CompressiveMetaData(9);
% CompressiveMeta.NumOfLostBScan = CompressiveMetaData(8);
% 
% % Remove the last column from each data matrix
% TruthData(:, end) = [];
% CompressiveData(:, end) = [];
% %% Upsample and Normalise
% 
% % Normalize each matrix to the range [0, 1]
% maxTruthData = max(max(TruthData));
% maxCompressiveData = max(max(CompressiveData));
% TruthData = TruthData ./ maxTruthData; 
% Compressive_norm = CompressiveData ./ maxCompressiveData;
% 
% % Upsampling rows for CscanCompressionRatio = 0.5
% rowUpsampleFactor = 1 / CompressiveMeta.CscanCompressionRatio; 
% CompressiveUpsampledRows = zeros(size(CompressiveData, 1) * rowUpsampleFactor, size(CompressiveData, 2));
% CompressiveUpsampledRows(1:rowUpsampleFactor:end, :) = CompressiveData;
% 
% % Upsampling columns for BscanCompressionRatio = 0.25
% colUpsampleFactor = 1 / CompressiveMeta.BscanCompressionRatio; 
% CompressiveUpsampled = zeros(size(CompressiveUpsampledRows, 1), size(CompressiveUpsampledRows, 2) * colUpsampleFactor);
% CompressiveUpsampled(:, 1:colUpsampleFactor:end) = CompressiveUpsampledRows;
% 
% % Normalize the upsampled matrix to the range [0, 1]
% CompressiveUpsampled_norm = (CompressiveUpsampled - min(CompressiveUpsampled(:))) / (max(CompressiveUpsampled(:)) - min(CompressiveUpsampled(:)));
% %% Get Mask, Y 
% A_2dMask = CompressiveUpsampled_norm ~= 0;
% % Linearise A
% A_1dMask = reshape(A_2dMask, [], 1);
% A_LinearIdx = find(A_1dMask == 1);
% x = reshape(CompressiveUpsampled_norm, [], 1);
% y = x(A_LinearIdx);
% %% Reconstruct and Compute the Error 
% [Estimation, reconstruction_time] = csAj(A_2dMask, y);
% reconstruction_error = RelErr(TruthData, Estimation);
% 
% % Scale back the values
% TruthData = TruthData .* maxTruthData; 
% Estimation = Estimation .* maxCompressiveData;
% %% Display & Save Results
% % Define the filename for storing error and time
% reconstructionResultFilename = [basePath, '_ReconstructionResult.csv'];
% % Create a table with reconstruction error and time
% resultsTable = table(reconstruction_error, reconstruction_time, 'VariableNames', {'ReconstructionError', 'ReconstructionTime'});
% % Write the table to a CSV file
% writetable(resultsTable, reconstructionResultFilename);
% % Display the reconstruction time and error
% disp(['Reconstruction Time: ', num2str(reconstruction_time), ' seconds']);
% disp(['Reconstruction Error: ', num2str(reconstruction_error), ' %']);
% 
% % Display the actual scanning times
% disp(['Full Scanning Time for Truth Data: ', num2str(TruthMeta.ActualScanningTimeSec), ' seconds']);
% disp(['Compressive Scanning Time for Compressive Data: ', num2str(CompressiveMeta.ActualScanningTimeSec), ' seconds']);
% 
% 
% % Save the Normalized TruthData Matrix
% fig1 = figure; imagesc(TruthData); axis equal; axis tight; title('TrueData'); colorbar;
% saveas(fig1, [basePath, 'TrueData.png']);
% 
% % Save the Normalized Compressive Matrix
% fig2 = figure; imagesc(Compressive_norm); axis equal; axis tight; title('SparseData'); colorbar;
% saveas(fig2, [basePath, 'SparseData.png']);
% 
% % Save the Normalized Upsampled Compressive Matrix
% fig3 = figure; imagesc(CompressiveUpsampled_norm); axis equal; axis tight; title('UpsampledSparseData'); colorbar;
% saveas(fig3, [basePath, 'UpsampledSparseData.png']);
% 
% % Save the Estimation
% fig4 = figure; imagesc(Estimation); axis equal; axis tight; title('Estimation'); colorbar;
% saveas(fig4, [basePath, 'Estimation.png']);