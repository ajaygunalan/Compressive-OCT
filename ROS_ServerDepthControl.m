rosshutdown;  clear all; clc; close all;
%%
rosinit('10.240.23.49');
service = rossvcserver('/estimate_depth', 'oct_msgs/Depth', @serviceCallback);
cleanupObj = onCleanup(@()rosshutdown);
disp('Service Server running. CTRL+C to exit.');

% Prompt the user
trialNum = input('Enter the trial number: ', 's');
scanNum = 1;

commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum, ' ', num2str(scanNum)];
[status, cmdout] = system(commandStr);
% Check the status and display the output or error message
if status == 0
    disp(['Command executed successfully: ', cmdout]);
else
    disp(['Error in executing command: ', cmdout]);
end


baseFolder = 'C:\Ajay_OCT\OCT-Guided-AutoCALM\data\getDepthFromSparse3Doct\';
folderLocation = fullfile(baseFolder, trialNum);
% Callback function
function resp = serviceCallback(~,~,resp)
    persistent before after scanNum;
    % Perform imaging
    scanNum = scanNum + 1;
    commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum, ' ', num2str(scanNum)];
    [status, cmdout] = system(commandStr);
    % Check the status and display the output or error message
    if status == 0
        disp(['Command executed successfully: ', cmdout]);
    else
        disp(['Error in executing command: ', cmdout]);
    end

    % Estimate depth
    before = readmatrix(['data\getDepthFromSparse3Doct\1\ScanNum_', num2str(scanNum - 1), '_Estimation.csv']);
    after = readmatrix(['data\getDepthFromSparse3Doct\1\ScanNum_', num2str(scanNum), '_Estimation.csv']);
    prefix = sprintf('%s_', scanNum);
    estimatedDepth = depthEstimation(after, before, prefix, folderLocation);

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("Depth estimated and sent: " + num2str(estimatedDepth));
end



