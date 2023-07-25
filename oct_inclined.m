%% 1. Load the Data
clear all;
close all;
clc;
% Define the file path
folderPath = 'data/oct_inclinded/line';  
fileName = 'OCTLaserBot/3.oct';
fullPath = fullfile(folderPath, fileName);
% Load the OCT Data
handle1 = OCTFileOpen(fullPath);
Intensity1 = OCTFileGetIntensity(handle1);
% Load depth image
frame1 = Intensity1;
figure; % Explicitly create a figure
im_obj1 = imagesc(frame1);
colormap gray;
axis off;
set(gca,'Position',[0 0 1 1]);
firstFrameName = sprintf('octWYS.png', fileName);
saveas(gcf, fullfile(folderPath, firstFrameName));
savedImageFullPath1 = fullfile(folderPath, firstFrameName);
close(gcf);

folderPath = 'data/oct_inclinded/line';  
fileName = 'GT/3.oct';
fullPath = fullfile(folderPath, fileName);
% Load the OCT Data
handle1 = OCTFileOpen(fullPath);
Intensity1 = OCTFileGetIntensity(handle1);
% Load depth image
frame1 = Intensity1;
figure; % Explicitly create a figure
im_obj1 = imagesc(frame1);
colormap gray;
axis off;
set(gca,'Position',[0 0 1 1]);
firstFrameName = sprintf('octTrue.png', fileName);
saveas(gcf, fullfile(folderPath, firstFrameName));
savedImageFullPath2 = fullfile(folderPath, firstFrameName);
close(gcf);
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
%% Estimate the depth form True Image. 

[depth_points, depth] = depthEstimationFrom2D(savedImageFullPath2);


