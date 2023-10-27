clear all, close all, clc;
%%
% Read the original image
filename = 'nature.jpg';  % Replace with the actual path to your image if it's not in the same directory
I = imread(filename);
I = rgb2gray(I); 

[rows, cols] = size(I);
I = preprocess(I, rows, cols);

% Obtain the dimensions of the image
[p, q] = size(I);  % p x q is the size of the image

% Create sensing matrix A with ones at alternate rows and columns, and zeros elsewhere
A = zeros(p, q);
A(1:2:end, 1:2:end) = 1;
A = logical(A);

% Displaying and saving the original resized image
imshow(I, []);
imwrite(I, 'x.png');

% Displaying and saving the sensing matrix A
figure;
imshow(A, []);
imwrite(A, 'A_2dMask.png');

% Element-wise multiplication to get y
y = A.*I;

% Displaying and saving the sampled image
figure;
imshow(y, []);
imwrite(y, 'y.png');

% Flatten the image I into a 1D vector x
x = I(:);

% Obtain the linear indices of the sampled pixels from the flattened version of A
sampler_linear_idx = find(A(:));

% Use these indices to extract the corresponding elements from x to form y_1d
y_1d = x(sampler_linear_idx);

% Save y_1d to a text file
writematrix(y_1d, 'MATLABy_1d.txt');
%%
% File: reconstruct_and_display.m

% Read the saved mask image
A_2dMask = imread('A_2dMask.png');
A_2dMask = A_2dMask(:,:,1);  % In case the image is read as an RGB image, take one channel
A_2dMask = logical(A_2dMask);  % Convert to logical type

% Read the saved y_1d vector
y_1d = readmatrix('PYTHONy_1d.txt');

% Call the csAj function with the read A_2dMask and y_1d
[reconstructed, t] = csAj(A_2dMask, y_1d);

% Get the size of the reconstructed image
[reconstructed_rows, reconstructed_cols] = size(reconstructed);

% Display the reconstructed image
figure;
imshow(reconstructed, []);
title('Reconstructed Image');

% Create text strings for the image size and reconstruction time
imageSizeText = sprintf('Size: %d x %d', reconstructed_rows, reconstructed_cols);
reconTimeText = sprintf('Time: %.2f seconds', t);
infoText = sprintf('%s\n%s', imageSizeText, reconTimeText);

% Create a text box with the information
annotation('textbox', [0.75, 0.5, 0.1, 0.1], 'String', infoText, 'FontSize', 12, ...
           'BackgroundColor', 'white', 'FitBoxToText', 'on');

% Save the reconstructed image
imwrite(reconstructed, 'reconstructed_image.png');
