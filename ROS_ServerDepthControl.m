rosshutdown;  clear all; clc; close all;
%%
% Initialize ROS with a specific master IP
rosinit('10.240.23.49');
 
% Define service server
service = rossvcserver('/estimate_depth', 'oct_msgs/Depth', @serviceCallback);

% Setup cleanup function to ensure rosshutdown is called when script ends
cleanupObj = onCleanup(@()rosshutdown);

disp('Service Server running. CTRL+C to exit.');

% Callback function
function resp = serviceCallback(~,~,resp)
    % Perform imaging
    % Prompt the user for the trial number
    trialNum = input('Enter the trial number: ', 's');
    scanNum = input('Enter the scan number: ', 's');
    commandStr = ['.\\bin\\x64\\Release\\OCTImageCapture.exe ', trialNum, ' ', scanNum];
    [status, cmdout] = system(commandStr);
    % Check the status and display the output or error message
    if status == 0
        disp(['Command executed successfully: ', cmdout]);
    else
        disp(['Error in executing command: ', cmdout]);
    end

    % Estimate depth
    % ... Depth estimation code goes here ...
    estimatedDepth = 5.0; % placeholder, replace with actual depth calculation

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("Depth estimated and sent: " + num2str(estimatedDepth));
end



