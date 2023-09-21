[depth_points, perpendicular_distances, final_image, T] = depthEstimationFrom2D('data\oct1.jpg');

% Plotting the final_image
figure;
imshow(final_image);
title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image');

% If you still want to save the table to a CSV file, you can do it here
writetable(T, 'ablate_surface_points.csv');
