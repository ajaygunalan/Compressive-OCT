clear all; close all;  clc;
%% 1. Load Data
folders = {'data/oct_inclinded/line', 'data/oct_inclinded/line', 'data/oct_inclinded/line', 'data/oct_inclinded/line', 'data/oct_inclinded/line', 'data/oct_inclinded/line'};
filenames = {'OCTLaserBot/1.oct', 'OCTLaserBot/2.oct', 'OCTLaserBot/3.oct', 'GT/1.oct', 'GT/2.oct', 'GT/3.oct'};
data = loadOCTData(folders, filenames);
%% 2. To find the angle roatted by WYS comapred to True
% Read the image
depthWYS = imread(savedImageFullPath1);

% Create a figure and an axes to hold the image
fig = figure;
ax = axes('Parent', fig);
imgHandle = imshow(depthWYS, 'Parent', ax);

% Create a coarse slider control
sliderCoarse = uicontrol('Parent', fig, 'Style', 'slider', 'Position', [150, 5, 300, 20],...
    'value', 0, 'min', -180, 'max', 180);

% Create a fine slider control
sliderFine = uicontrol('Parent', fig, 'Style', 'slider', 'Position', [150, 35, 300, 20],...
    'value', 0, 'min', -1, 'max', 1);

% Store the image data, handle, and sliders using guidata
handles = guidata(fig);
handles.depthWYS = depthWYS;
handles.imgHandle = imgHandle;
handles.sliderCoarse = sliderCoarse;
handles.sliderFine = sliderFine;
guidata(fig, handles);

% Add a listener to the sliders
addlistener(sliderCoarse, 'ContinuousValueChange', @(src, event) rotateImage(fig));
addlistener(sliderFine, 'ContinuousValueChange', @(src, event) rotateImage(fig));
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




