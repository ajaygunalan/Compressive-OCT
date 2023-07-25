function [depth_points, perpendicular_distances] = depthEstimationFrom2D(savedImageFullPath)
    %% 1. Detect the ablate surface -> Binary, Dilate, Erode, edge, Contnour. 
    depthI = imread(savedImageFullPath);

    % Check if the image is RGB
    if ndims(depthI) == 3
        % Convert RGB to grayscale
        depthI = rgb2gray(depthI);
    end

    % Convert the intensity data to binary image
    BW = imbinarize(depthI, 'adaptive');

    % Perform dilation + erosion to close gaps in between object edges
    se = strel('disk',5);
    BW = imdilate(BW,se);
    BW = imerode(BW,se);

    % Edge detection using Canny method
    % Set thresholds for edge detection
    lowerThreshold = 0.8; % Lower threshold for edge detection, can adjust this value
    upperThreshold = 0.85; % Upper threshold for edge detection, can adjust this value
    sigma = 2;

    BW = edge(BW, 'Canny', [lowerThreshold upperThreshold], sigma);

    % Find contours (boundaries in this case)
    boundaries = bwboundaries(BW);
    surface = cell(1, length(boundaries)); % Initialize surface as cell array

    % Initialize a threshold for boundary size
    sizeThreshold = 1000; % You can adjust this value

    % Draw contours on a copy of the original frame
    frame_contours = depthI;
    surface_idx = 0; % Index to track contours added to surface
    for k = 1:length(boundaries)
        boundary = boundaries{k};
        if length(boundary) > sizeThreshold
            surface_idx = surface_idx + 1;
            surface{surface_idx} = boundary;
            frame_contours(sub2ind(size(frame_contours), boundary(:,1), boundary(:,2))) = 255;
        end
    end
    surface = surface(1:surface_idx); % Remove unused cells

    % Visualize the image with contours
    figure;
    imshow(frame_contours, []);
    title('Contour Image');


    %% 2. Approximate contour points with Line Segments
    % Initialize cell array to hold the reduced surfaces and the ablate surfaces
    surface_reduced = cell(1, surface_idx);
    ablate_surface = cell(1, surface_idx); % Initialize ablate_surface cell array

    % Loop over each surface
    for idx = 1:surface_idx
        % Reduce the polygon using Douglas-Peucker algorithm
        tolerance = 0.02; % you can adjust this
        surface_reduced{idx} = reducepoly(surface{idx}, tolerance);
    end
    

    
    
    % Overlay the reduced surfaces on the original image
    frame_contours_with_reduced = depthI;
    ablate_surface_idx = 0; % Initialize ablate_surface index
    for idx = 1:surface_idx
        % Calculate dx for the first line segment in the reduced surface
        dx = abs(surface_reduced{idx}(2,2) - surface_reduced{idx}(1,2));
    
        % If dx is less than 2, add the current contour to ablate_surface and skip to the next iteration
        if dx > 2
            ablate_surface_idx = ablate_surface_idx + 1;
            ablate_surface{ablate_surface_idx} = surface_reduced{idx};
            continue;
        end
    
        frame_contours_with_reduced(sub2ind(size(frame_contours_with_reduced), round(surface_reduced{idx}(:,1)), round(surface_reduced{idx}(:,2)))) = 255;
    end

    % Visualize the image with reduced contours
    figure;
    imshow(frame_contours_with_reduced, []);
    hold on;
    
    % remove close points
    ablate_surface_old = ablate_surface;
    minDistance = 3.6; % adjust this as needed
    for idx = 1:ablate_surface_idx
        ablate_surface{idx} = filterClosePoints(ablate_surface{idx}, minDistance);
    end


    % Draw each ablate surface
    for idx = 1:ablate_surface_idx
        line(ablate_surface{idx}(:,2), ablate_surface{idx}(:,1), 'color','r','linewidth',1.5,'marker','o','markersize',5);
    
        % Annotate the surface with surface_idx value
        mid_point = round(size(ablate_surface{idx}, 1)/2);
        text(ablate_surface{idx}(mid_point, 2), ablate_surface{idx}(mid_point, 1), num2str(idx), 'Color', 'yellow', 'FontSize', 14);
    end
    title('Original and Ablate Contour Image');
 %% 3. Combine the contour and find depth
    % Filter out empty matrices
    ablate_surface = ablate_surface(~cellfun(@isempty, ablate_surface));

    % Concatenate all the ablate_surface into one list
    all_surfaces = vertcat(ablate_surface{:});

    % Apply reducepoly function
    tolerance = 0.002;  % Set the tolerance
    reduced_all_surfaces = reducepoly(all_surfaces, tolerance);

    % Get the first two and last two points from reduced_all_surfaces
    first_two_points = reduced_all_surfaces(1:2,:);
    last_two_points = reduced_all_surfaces(end-1:end,:);
    top_layer_points = [first_two_points; last_two_points];

    % Define threshold and filter depth_points based on change in x-coordinate
    threshold = 10; % Set your threshold
    depth_points = reduced_all_surfaces(3:end-2,:);
    dX = [threshold+1; abs(diff(depth_points(:, 2)))]; % Calculate dX, ensuring first point is kept
    depth_points = depth_points(dX > threshold, :);

    % Combine these points back into one array
    reduced_all_surfaces = [first_two_points; depth_points; last_two_points];

    % Calculate the slope and intercept of the Top Layer line
    slope = (last_two_points(2,1) - first_two_points(1,1)) / (last_two_points(2,2) - first_two_points(1,2));
    intercept = first_two_points(1,1) - slope * first_two_points(1,2);

    % Calculate the perpendicular distances from depth_points to the Top Layer line
    perpendicular_distances = abs(slope * depth_points(:,2) - depth_points(:,1) + intercept) / sqrt(slope^2 + 1);

    % Find the intersection points on the green line
    x_intersect = (depth_points(:,2) + slope * depth_points(:,1) - slope * intercept) / (slope^2 + 1);
    y_intersect = slope * x_intersect + intercept;

    % Create a copy of the original image to plot on
    overlay_image = depthI;

    % Overlay the original and reduced contours on the image
    figure;
    imshow(overlay_image, []);
    hold on;

    % Plot the reduced all surfaces
    line(reduced_all_surfaces(:,2), reduced_all_surfaces(:,1), 'color', 'r', 'marker', 'o');

    % Draw the line representing the "Top Layer"
    line(top_layer_points(:,2), top_layer_points(:,1), 'color', 'g', 'LineWidth', 2);

    % Plot the shortest distances from depth_points to the Top Layer line
    numDepthPoints = size(depth_points, 1); % Compute once outside loop
    for i = 1:numDepthPoints
        line([depth_points(i,2), x_intersect(i)], [depth_points(i,1), y_intersect(i)], 'color', 'y', 'LineWidth', 1);
    end

    % Set title
    title('Ablated Surfaces (Red), Top Layer (Green), and Vertical Distances (Yellow) on Depth Image');

    % Keep the hold off
    hold off;
end

















% %% 4. Combine all the contour to form a single contour.
% 
% % Filter out empty matrices
% ablate_surface = ablate_surface(~cellfun(@isempty, ablate_surface));
% 
% % Concatenate all the ablate_surfac into one list
% all_surfaces = vertcat(ablate_surface{:});
% 
% % Apply reducepoly function
% tolerance = 0.002;  % Set the tolerance
% reduced_all_surfaces = reducepoly(all_surfaces, tolerance);
% 
% % Sort reduced_all_surfaces by the x-coordinate (column 2) in increasing order
% reduced_all_surfaces = sortrows(reduced_all_surfaces, 2);
% 
% 
% % Get the first two and last two points from reduced_all_surfaces
% first_two_points = reduced_all_surfaces(1:2,:);
% last_two_points = reduced_all_surfaces(end-1:end,:);
% depth_points = reduced_all_surfaces(3:end-2,:);
% 
% threshold = 10; % Set your threshold
% dX = abs(diff(depth_points(:, 2))); % Calculate dX
% 
% % Keep only the points where dX > threshold
% depth_points = depth_points([true; dX > threshold], :);
% 
% reduced_all_surfaces = [first_two_points; depth_points; last_two_points];
% 
% % Combine these points into one array
% top_layer_points = [first_two_points; last_two_points];
% 
% 
% %% Draw the top surface. 
% 
% % Create a copy of the original image to plot on
% overlay_image = depthI;
% 
% 
% 
% % Overlay the original and reduced contours on the image
% figure;
% imshow(overlay_image, []);
% hold on;
% 
% % Plot the original ablated surfaces
% for i = 1:numel(ablate_surface)
%     line(ablate_surface{i}(:,2), ablate_surface{i}(:,1), 'color', 'b', 'marker', 'o');
% end
% 
% % Plot the reduced all surfaces
% line(reduced_all_surfaces(:,2), reduced_all_surfaces(:,1), 'color', 'r', 'marker', 'o');
% 
% % Draw the line representing the "Top Layer"
% line(top_layer_points(:,2), top_layer_points(:,1), 'color', 'g', 'LineWidth', 2);
% 
% % Set title
% title('Original (Blue), Reduced (Red) Ablated Surfaces, and Top Layer (Green) on Depth Image');
% 
% % Keep the hold off
% hold off;
% %%
% depth_points = reduced_all_surfaces(3:end-2,:);
% % Get the first and last points from reduced_all_surfaces
% first_point = reduced_all_surfaces(1,:);
% last_point = reduced_all_surfaces(end,:);
% 
% % Calculate the slope and intercept of the Top Layer line
% slope = (last_point(1) - first_point(1)) / (last_point(2) - first_point(2));
% intercept = first_point(1) - slope * first_point(2);
% 
% % Calculate the vertical distances from depth_points to the Top Layer line
% vertical_distances = abs(slope * depth_points(:,2) - depth_points(:,1) + intercept) / sqrt(slope^2 + 1);
% 
% % Create a copy of the original image to plot on
% overlay_image = depthI;
% 
% % Overlay the original and reduced contours on the image
% figure;
% imshow(overlay_image, []);
% hold on;
% 
% % Plot the original ablated surfaces
% for i = 1:numel(ablate_surface)
%     line(ablate_surface{i}(:,2), ablate_surface{i}(:,1), 'color', 'b', 'marker', 'o');
% end
% 
% % Plot the reduced all surfaces
% line(reduced_all_surfaces(:,2), reduced_all_surfaces(:,1), 'color', 'r', 'marker', 'o');
% 
% % Draw the line representing the "Top Layer"
% line([first_point(2), last_point(2)], [first_point(1), last_point(1)], 'color', 'g', 'LineWidth', 2);
% 
% % Plot the vertical distances from depth_points to the Top Layer line
% for i = 1:size(depth_points, 1)
%     line([depth_points(i,2), depth_points(i,2)], [depth_points(i,1), depth_points(i,1) - vertical_distances(i)], 'color', 'y', 'LineWidth', 1);
% end
% 
% % Set title
% title('Original (Blue), Reduced (Red) Ablated Surfaces, Top Layer (Green), and Vertical Distances (Yellow) on Depth Image');
% 
% % Keep the hold off
% hold off;
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% %% Apply some checks to filter out wrong contour detection.
% figure;
% imshow(frame_contours_with_reduced, []);
% hold on;
% 
% % Draw each reduced surface and calculate and plot the gradient
% for idx = 1:surface_idx
%     % Draw reduced surface
%     line(surface_reduced{idx}(:,2), surface_reduced{idx}(:,1), 'color','r','linewidth',1.5,'marker','o','markersize',5);
%     
%     % Calculate the gradient
%     [dx, dy] = gradient(surface_reduced{idx});
%     
%     % Approximate gradient by dy/dx. Avoid division by zero.
%     grad = dy ./ (dx + (dx == 0));
%     
%     % Normalize gradient to [0, 1] for visualization
%     grad = (grad - min(grad)) / (max(grad) - min(grad));
%     
%     % Plot gradient (represented by color) on reduced surface
%     scatter(surface_reduced{idx}(:,2), surface_reduced{idx}(:,1), [], grad, 'filled');
% end
% 
% colorbar; % Show colorbar for reference
% title('Original, Reduced Contour Image, and Gradient');
% %% Find Top Layer from the Contour
% % Get the first 10 points
% first_10_points = surface(1:10, :);
% 
% % Compute the average of these points
% average_point1 = mean(first_10_points);
% 
% % Get the last 10 points
% last_10_points = surface(end-9:end, :);
% 
% % Compute the average of these points
% average_point2 = mean(last_10_points);
% 
% % Find the total mean by averaging the two averages
% total_average_point = mean([average_point1; average_point2]);
% 
% % Display the total average point
% disp(['The total average point is at (', num2str(total_average_point(1)), ', ', num2str(total_average_point(2)), ').']);
% 
% figure;
% imshow(depthI); % Display the original image
% hold on; % Keep the image, add further plotting on top
% plot([1, size(depthI, 2)], [total_average_point(1), total_average_point(1)], 'r', 'LineWidth', 2); % Draw a horizontal line
% hold off; % Release the figure for further plotting
% title('Image with a horizontal line');
% %%
% % Compute the gradient (difference between adjacent points)
% gradientY = diff(surface(:,1));
% gradientX = diff(surface(:,2));
% 
% % Compute the actual gradient (rise/run)
% gradientContour = gradientY ./ gradientX;
% 
% % Replace Inf and NaN with max non-Inf value
% maxVal = max(gradientContour(~isinf(gradientContour) & ~isnan(gradientContour)));
% gradientContour(isinf(gradientContour) | isnan(gradientContour)) = maxVal;
% 
% % Normalize the gradient to the range [0, 1] for color mapping
% gradientNorm = (gradientContour - min(gradientContour)) / (max(gradientContour) - min(gradientContour));
% 
% % Create a new figure for the image
% figure;
% imshow(depthI);
% hold on;
% 
% % Draw the contour with color based on gradient
% for i = 1:(length(surface) - 1) % Subtract 1 because gradient is one element shorter than surface
%     plot(surface(i:i+1,2), surface(i:i+1,1), 'LineWidth', 8, 'Color', [gradientNorm(i) 0 1-gradientNorm(i)]);
% end
% 
% title('Contour on the Image with Gradient Colors');
% hold off;
% 
% 
% 
% 
% %%
% 
% 
% % Assuming 'surface' is your boundary (a Nx2 matrix)
% % Assuming 'frame_contours' is your image
% 
% % The top1 boundary point is the one with the smallest row index
% [~, top_idx] = min(surface(:,1));
% top_boundary = surface(top_idx, :);
% 
% % The left boundary point is the one with the smallest column index
% [~, left_idx] = min(surface(:,2));
% left_boundary = surface(left_idx, :);
% 
% % The right boundary point is the one with the largest column index
% [~, right_idx] = max(surface(:,2));
% right_boundary = surface(right_idx, :);
% 
% % Visualize the image with contours and boundary points
% figure;
% imshow(frame_contours, []);
% hold on;
% plot(top_boundary(2), top_boundary(1), 'r*', 'MarkerSize', 10);   % Plot as red star
% plot(left_boundary(2), left_boundary(1), 'g*', 'MarkerSize', 10); % Plot as green star
% plot(right_boundary(2), right_boundary(1), 'b*', 'MarkerSize', 10);% Plot as blue star
% hold off;
% title('Contour Image with Boundary Points');
% 
% 
% %% Find the left and right boubdaries of ablataion
% %% Find Distance from countour pts to top layer
% %%






% %%
% % Find crests and troughs based on grayscale intensity
% % Smoothen the image before finding local maxima and minima to reduce noise
% frame_smooth = imgaussfilt(frame, 2);
% 
% % Find local maxima
% crests = imregionalmax(frame_smooth);
% 
% % Find local minima
% troughs = imregionalmin(frame_smooth);
% 
% % Display the results
% figure;
% imshow(frame_contours, []);
% hold on;
% title('Contours, Crests, and Troughs');
% 
% % Draw crests (red) and troughs (blue)
% [r, c] = find(crests);
% plot(c, r, 'r*');
% [r, c] = find(troughs);
% plot(c, r, 'b*');
% 
% 
% %% Depth Estimation from Intensity
% % Read the frame
% frame = Intensity1;
% 
% % Define the conversion factor um per pixel
% pixels_per_unit_x = 11.62;
% pixels_per_unit_z = 3.46; 
% 
% % Display the frame
% figure;
% im_obj = imagesc(frame);
% colormap(gray);
% axis off;
% set(gca, 'Position', [0 0 1 1]);
% 
% % Draw a line on the frame
% title('Draw a line on the image and double-click to finish');
% h_line = imline;
% 
% % Get the line's position (endpoints)
% line_pos = h_line.getPosition;
% 
% % Calculate the length of the line in pixels
% x_diff = (line_pos(2,1) - line_pos(1,1))*pixels_per_unit_x;
% z_diff = (line_pos(2,2) - line_pos(1,2))*pixels_per_unit_z;
% line_length_units = sqrt(x_diff^2 + z_diff^2);
% line_length_units= line_length_units/1000; %convert um into mm
% 
% % Display the length of the line on the frame
% midpoint = [mean(line_pos(:,1)), mean(line_pos(:,2))];
% text(midpoint(1), midpoint(2), sprintf('Length: %.2f mm', line_length_units), ...
%      'Color', 'r', 'FontSize', 12, 'HorizontalAlignment', 'center', ...
%      'VerticalAlignment', 'bottom');
% 
% % (Optional) Display the length in a message box
% msgbox(sprintf('Line length: %.2f mm', line_length_units), 'Line Length');
% 
% % %% Assuming that RGB data is present in the OCT file, let's try to get it
% % VideoImage1 = OCTFileGetColoredData(handle1, 'VideoImage');
% % figure;
% % imshow(VideoImage1);
% % title('RGB');
% % 
% % % Save the image with a dynamic filename
% % VideoImageName = sprintf('VideoImage_%s.png', fileName);
% % imwrite(VideoImage1, fullfile(folderPath, VideoImageName));
