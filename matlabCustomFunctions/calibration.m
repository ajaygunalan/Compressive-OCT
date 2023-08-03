clear all; close all;  clc;
%% 1. Load Data
base_folder = '..\data\oct_inclinded\line'; % relative path to the data folder
folders = {base_folder, base_folder, base_folder, base_folder, base_folder, base_folder};
filenames = {'OCTLaserBot\1.oct', 'OCTLaserBot\2.oct', 'OCTLaserBot\3.oct', 'GT\1.oct', 'GT\2.oct', 'GT\3.oct'};
data = loadOCTData(folders, filenames);
%% 2. Estimate the depth. 

% Initialize structure to hold depth data
for i = 1:numel(data)
    [depth_points, depth, final_image] = depthEstimationFrom2D(data(i).imagePath);
    depthData(i).depthPoints = depth_points;
    depthData(i).depth = depth;
    depthData(i).finalImage = final_image; % Add final image to the depthData structure
end

% Plotting all images
figure;

numOCTImages = sum(contains({data.imagePath}, 'OCTLaserBot'));
numGTImages = sum(contains({data.imagePath}, 'GT'));

numRows = max(numOCTImages, numGTImages); % Number of rows is based on the maximum of the two

% Assuming depthData is ordered the same way as data
OCTImages = depthData(contains({data.imagePath}, 'OCTLaserBot'));
GTImages = depthData(contains({data.imagePath}, 'GT'));

for i = 1:numRows
    if i <= numOCTImages
        % Plot OCTLaserBot images on left
        subplot(numRows, 2, i * 2 - 1);
        imshow(OCTImages(i).finalImage);
        title(['OCTLaserBot Image ', num2str(i)]);
    end

    if i <= numGTImages
        % Plot GT images on right
        subplot(numRows, 2, i * 2);
        imshow(GTImages(i).finalImage);
        title(['GT Image ', num2str(i)]);
    end
end
%% Calibrataion & Verfication
% calibFactor = GT/OCTLaserBot;
% calibFactor1 = depthData(4).depth/depthData(2).depth;
% calibFactor2 = depthData(6).depth/depthData(3).depth;

% Define pairs of depth data to calculate calibration factors
depthPairs = [4 2; 6 3]; % Adjust this as needed

% Calculate calibration factors in a loop
calibFactors = zeros(size(depthPairs, 1), 1);
for i = 1:size(depthPairs, 1)
    calibFactors(i) = depthData(depthPairs(i, 1)).depth / depthData(depthPairs(i, 2)).depth;
end

% Calculate mean 
meanCalib = mean(calibFactors);

% Calculate median
medianCalib = median(calibFactors);

% Calculate standard deviation
stdCalib = std(calibFactors);

% Calculate and print individual errors
errors = zeros(size(depthPairs, 1), 1);
calculatedDepth = zeros(size(depthPairs, 1), 1);
for i = 1:size(depthPairs, 1)
    calculatedDepth(i) = depthData(depthPairs(i, 2)).depth * meanCalib;
    actualDepth = depthData(depthPairs(i, 1)).depth;
    errors(i) = calculatedDepth(i) - actualDepth;
    fprintf('For WYS %d (depth: %f) and GT %d (depth: %f), Estimated depth: %f, Error: %f\n', depthPairs(i, 2), depthData(depthPairs(i, 2)).depth, depthPairs(i, 1), depthData(depthPairs(i, 1)).depth, calculatedDepth(i), errors(i));
end

% Calculate and print mean error
meanError = mean(errors);
fprintf('Mean error: %f\n', meanError);

% Display results for calibration factors
fprintf('Mean of Calibration Factors: %f\n', meanCalib);
fprintf('Median of Calibration Factors: %f\n', medianCalib);
fprintf('Standard Deviation of Calibration Factors: %f\n', stdCalib);
%% To find the angle roatted by WYS comapred to True
% Read the image
% depthWYS = imread(savedImageFullPath1);
% 
% % Create a figure and an axes to hold the image
% fig = figure;
% ax = axes('Parent', fig);
% imgHandle = imshow(depthWYS, 'Parent', ax);
% 
% % Create a coarse slider control
% sliderCoarse = uicontrol('Parent', fig, 'Style', 'slider', 'Position', [150, 5, 300, 20],...
%     'value', 0, 'min', -180, 'max', 180);
% 
% % Create a fine slider control
% sliderFine = uicontrol('Parent', fig, 'Style', 'slider', 'Position', [150, 35, 300, 20],...
%     'value', 0, 'min', -1, 'max', 1);
% 
% % Store the image data, handle, and sliders using guidata
% handles = guidata(fig);
% handles.depthWYS = depthWYS;
% handles.imgHandle = imgHandle;
% handles.sliderCoarse = sliderCoarse;
% handles.sliderFine = sliderFine;
% guidata(fig, handles);
% 
% % Add a listener to the sliders
% addlistener(sliderCoarse, 'ContinuousValueChange', @(src, event) rotateImage(fig));
% addlistener(sliderFine, 'ContinuousValueChange', @(src, event) rotateImage(fig));



