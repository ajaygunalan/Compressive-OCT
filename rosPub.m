% Initialize ROS with a specific master IP
rosinit('10.240.23.63');

% Create a publisher to a topic
depth_pub = rospublisher('/depth_values', 'std_msgs/Float64');

% Create a message
depth_msg = rosmessage(depth_pub);

% Setup cleanup function to ensure rosshutdown is called when script ends
cleanupObj = onCleanup(@()rosshutdown);

% Keep publishing until CTRL+C is pressed
disp('Starting to publish depth values. Press CTRL+C to stop...');
try
    while true
        % Fill in data
        depth_msg.Data = 5.0; % Change this to your depth value

        % Publish message
        send(depth_pub, depth_msg);
        
        % Optional: pause for a while
        pause(0.1);
    end
catch ME
    if ~strcmp(ME.identifier,'MATLAB:mir_interrupt')
        rethrow(ME);
    end
end

disp('Stopped publishing.');



