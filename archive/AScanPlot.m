% File path to the CSV
file_path = 'path/to/your/csv/file.csv';

% Read the CSV file as a plain text file to extract the numbers
fid = fopen(file_path, 'r');
raw_data = textscan(fid, '%s', 'Delimiter', ',');
fclose(fid);

% Filter out any empty strings and convert the rest to float
float_data = cellfun(@str2double, raw_data{1});
float_data = float_data(~isnan(float_data));

% Generate the corresponding depth values based on given SpacingZ
SpacingZ = 0.003455;
depth_values = 0:SpacingZ:(length(float_data)-1)*SpacingZ;

% Plot the data
figure('Position', [10, 10, 1000, 600])
plot(depth_values, float_data);
xlabel('Depth (mm)');
ylabel('Intensity (dB)');
title('Depth vs Intensity');
