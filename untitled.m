[depth_points, edDstimateepth, final_image] = depthEstimationFrom2D('data\oct1.jpg');

% Plotting the final_image
figure;
imshow(final_image);
title('Final Image for data\oct1.jpg');
