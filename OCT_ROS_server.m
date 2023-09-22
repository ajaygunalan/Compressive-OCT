% Clear all variables, close all figures, clear command window
clear all; close all; clc;
global counter folderPath;
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
    global counter folderPath;
       
    disp('OCT Server: CALM client is requestsing depth.');
    
    if isempty(counter)
        counter = 1;  % Initialize counter if empty
        folderPath = input(['Please enter the folder path (e.g., "data/salmone/' ...
            '"): '], 's');  % Ask for folder path
    else
        counter = counter + 1;  % Increment counter
    end

   
    userResponse = input('OCT Server: Kindly perform imagaing using GUI. Type "ok" when done, anything else to quit.', 's');
    
    if ~strcmp(userResponse, 'ok')
        fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    % Perform imaging
%     BScanRangeMM = 6.81;
%     ShiftX = 1.02;
%     ShiftY = -0.87;
%     Angle_deg = -51.37;
%     cmd = sprintf('".\\bin\\x64\\Debug\\OCTImageCapture.exe" %f %f %f %f', BScanRangeMM, ShiftX, ShiftY, Angle_deg);
%     system(cmd);
%     disp('OCT Server: Imaging done.');
%     User checks the image manually
%     userResponse = input('\nOCT Server: Please check the images. Type "ok" to continue if the images are OK, anything else to quit.', 's');
%     if ~strcmp(userResponse, 'ok')
%         fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
%         rosshutdown;
%         return;
%     end

    % Estimate depth
    % ... Depth estimation code goes here ...
    disp('OCT Server: Estimating depth from OCT Images.');
    % Generate the variable filename based on counter and folderPath
    filename = [folderPath, 'oct', num2str(counter), '.jpg'];

    % Perform imaging and depth estimation
    [depth_points, depth, final_image] = depthEstimationFrom2D(filename);

    % Plotting the final_image
    figure;
    imshow(final_image);
    title('Final Image');


    % Store the user's estimated depth in a variable
    estimatedDepth = depth;

    % User checks the estimated depth manually
    userResponse = input(['OCT Server: Estimated depth: ', num2str(estimatedDepth), ' mm. \nType "ok" to continue if the depth is OK, anything else to quit.'], 's');

    if ~strcmp(userResponse, 'ok')
        fprintf('OCT Server: Execution cancelled by the user. Exiting...\n');
        rosshutdown;
        return;
    end

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("OCT Server: Depth estimated and sent: " + num2str(estimatedDepth));
end






