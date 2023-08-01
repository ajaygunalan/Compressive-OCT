% Clear all variables, close all figures, clear command window
clear all; close all; clc;
rosshutdown;

% Initialize ROS with a specific master IP
rosinit('10.240.23.63');

% Define service server
service = rossvcserver('/estimate_depth', 'oct_msgs/Depth', @serviceCallback);

% Setup cleanup function to ensure rosshutdown is called when script ends
cleanupObj = onCleanup(@()rosshutdown);

disp('Service Server running. CTRL+C to exit.');

% Callback function
function resp = serviceCallback(~,~,resp)
    % Perform imaging
    % system('".\bin\x64\Debug\OCTImageCapture.exe"'); % Call .exe file to capture image

    % User checks the image manually
    userResponse = input('Please check the images. Type "ok" to continue if the images are OK, anything else to quit.', 's');

    if ~strcmp(userResponse, 'ok')
        fprintf('\nExecution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    % Estimate depth
    % ... Depth estimation code goes here ...
    estimatedDepth = 5.0; % placeholder, replace with actual depth calculation

    % User checks the estimated depth manually
    userResponse = input(['Estimated depth: ', num2str(estimatedDepth), '. Type "ok" to continue if the depth is OK, anything else to quit.'], 's');

    if ~strcmp(userResponse, 'ok')
        fprintf('\nExecution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("Depth estimated and sent: " + num2str(estimatedDepth));
end






