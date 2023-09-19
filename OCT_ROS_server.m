% Clear all variables, close all figures, clear command window
clear all; close all; clc;
rosshutdown;

% Initialize ROS with a specific master IP
rosinit('10.240.23.63');

% Define service server
service = rossvcserver('/estimate_depth', 'oct_msgs/Depth', @serviceCallback);

% Setup cleanup function to ensure rosshutdown is called when script ends
cleanupObj = onCleanup(@()rosshutdown);

disp('OCT Server (ROS Service) running. CTRL+C to exit.');

% Callback function
function resp = serviceCallback(~,~,resp)
    disp('OCT Server: CALM client is requestsing depth.');

    userResponse = input('\nOCT Server: Type "ok" to perform imaging, anything else to quit.', 's');
    
    if ~strcmp(userResponse, 'ok')
        fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end
  
    % Perform imaging
    BScanRangeMM = 5.22;
    ShiftX = -0.24;
    ShiftY = -0.34;
    Angle_deg = -40.53;
    cmd = sprintf('".\\bin\\x64\\Debug\\OCTImageCapture.exe" %f %f %f %f', BScanRangeMM, ShiftX, ShiftY, Angle_deg);
    system(cmd);
    disp('OCT Server: Imaging done.');

    % User checks the image manually
    userResponse = input('\nOCT Server: Please check the images. Type "ok" to continue if the images are OK, anything else to quit.', 's');

    if ~strcmp(userResponse, 'ok')
        fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    % Estimate depth
    % ... Depth estimation code goes here ...
    estimatedDepth = 5.0; % placeholder, replace with actual depth calculation

    % User checks the estimated depth manually
    userResponse = input(['\nOCT Server: Estimated depth: ', num2str(estimatedDepth), '. Type "ok" to continue if the depth is OK, anything else to quit.'], 's');

    if ~strcmp(userResponse, 'ok')
        fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("OCT Server: Depth estimated and sent: " + num2str(estimatedDepth));
end






