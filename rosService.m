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
    % ... Imaging code goes here ...

    % Estimate depth
    % ... Depth estimation code goes here ...
    estimatedDepth = 5.0; % placeholder, replace with actual depth calculation

    resp.Depth = rosmessage('std_msgs/Float64');
    resp.Depth.Data = estimatedDepth;
    disp("Depth estimated and sent: " + num2str(estimatedDepth));
end




