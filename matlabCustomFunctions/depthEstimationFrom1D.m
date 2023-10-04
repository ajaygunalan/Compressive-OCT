function [depth] = depthEstimationFrom1D(folderPath, filename)
    % Construct the full file path
    fullFilePath = fullfile(folderPath, filename);

    % Read the CSV file into a matrix
    data_matrix = csvread(fullFilePath);

    % Calculate the average of each column
    avg_data = mean(data_matrix, 1);

    % Generate the corresponding depth values based on given SpacingZ
    SpacingZ = 0.003455;
    depth_values = 0:SpacingZ:(length(avg_data) - 1) * SpacingZ;

    % Gaussian smoothing
    std_dev = 5;
    smoothed_data = imgaussfilt(avg_data, std_dev);

    % Find the peak intensity and corresponding depth value
    [peak_intensity, peak_index] = max(smoothed_data);
    depth = depth_values(peak_index);

    % Plot the original and smoothed data
    fig = figure('Position', [10, 10, 1000, 600], 'Visible', 'off');
    hold on;
    plot(depth_values, avg_data, 'DisplayName', 'Average Intensity');
    plot(depth_values, smoothed_data, 'DisplayName', 'Gaussian Smoothing', 'LineWidth', 2);

    % Indicate the peak point on the plot
    scatter(depth, peak_intensity, 'black', 'filled', 'DisplayName', 'Depth');
    text(depth, peak_intensity, ['  (', num2str(depth), ' mm, ', num2str(peak_intensity), ' dB)'], 'FontSize', 12, 'Color', 'black');

    xlabel('Depth (mm)');
    ylabel('Average Intensity (dB)');
    title(['Depth vs Average Intensity (Peak Indicated)']);
    legend;
    hold off;

    % Save the plot as an image
    [~, name, ~] = fileparts(filename);  % Get the name dynamically
    final_image_filename = fullfile(folderPath, [name '_final.png']);  % Use PNG format

    
    saveas(fig, final_image_filename);
end
