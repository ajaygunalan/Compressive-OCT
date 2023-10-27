% File: reconstruct_and_display.m

% Read the saved mask image
A_2dMask = imread('A_2dMask.png');
A_2dMask = A_2dMask(:,:,1);  % In case the image is read as an RGB image, take one channel
A_2dMask = logical(A_2dMask);  % Convert to logical type

% Read the saved y_1d vector
y_1d = dlmread('y_1d.txt');

% Call the csAj function with the read A_2dMask and y_1d
[reconstructed, t] = csAj(A_2dMask, y_1d);

% Display the reconstructed image
figure;
imshow(reconstructed, []);
title('Reconstructed Image');

% Save the reconstructed image
imwrite(reconstructed, 'reconstructed_image.png');