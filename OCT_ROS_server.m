clear all; close all; clc;
global counter folderPath;
rosshutdown;

folderPath = "data/plaster/";
rosinit('10.240.23.63');
service = rossvcserver('/estimate_depth', 'oct_msgs/Depth', @serviceCallback);
cleanupObj = onCleanup(@()rosshutdown);

disp('OCT Server (ROS Service) running. CTRL+C to exit.');

counter = 1;
function resp = serviceCallback(~,~,resp)
    global counter folderPath;
       
    disp('OCT Server: CALM client is requesting depth.');
    
    while true
        userResponse = input('OCT Server: Kindly perform imaging using GUI. Type "ok" when done, anything else to quit.', 's');
        
        if ~strcmp(userResponse, 'ok')
            fprintf('\nOCT Server: Execution cancelled by the user. Exiting...\n');
            rosshutdown;
            return;
        end

         counterInput = input('OCT Server: Please enter the counter number: ', 's');
        counter = str2double(counterInput);  % Convert string to double

        disp('OCT Server: Estimating depth from OCT Images.');
        filename = ['oct', num2str(counter), '.jpg'];
        [depth_points, perpendicular_distances] = depthEstimationFrom2D(folderPath, filename);

        estimatedDepth = perpendicular_distances;

        % Display the final image
        [~, name, ext] = fileparts(filename);
        final_image_filename = fullfile(folderPath, [name 'final' ext]);
        figure;
        imshow(imread(final_image_filename));
        title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image');

        userResponse = input(['OCT Server: Estimated depth: ', num2str(estimatedDepth), ' mm. \nType "ok" to continue if the depth is OK, anything else to re-perform imaging.'], 's');

        if strcmp(userResponse, 'ok')
            break;
        else
            disp('OCT Server: Re-performing imaging as per user request.');
        end
    end

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("OCT Server: Depth estimated and sent: " + num2str(estimatedDepth));

    counter = counter + 1;
end
