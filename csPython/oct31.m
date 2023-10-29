clear all; close all; clc;
%%
A_2dMask = readmatrix('Python_A.txt');
y_1d = readmatrix('Python_y.txt');

A_2dMask = logical(A_2dMask);  % Convert to logical type
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
%%

