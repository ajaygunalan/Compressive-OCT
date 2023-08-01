function [depth_points, perpendicular_distances, final_image] = depthEstimationFrom2D(savedImageFullPath)
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
    sizeThreshold = 800; % You can adjust this value
    
    
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
%     figure;
%     imshow(frame_contours, []);
%     title('Contour Image');


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
%     figure;
%     imshow(frame_contours_with_reduced, []);
%     hold on;
    
    % remove close points
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
%     figure;
%     imshow(overlay_image, []);
%     hold on;

    % Save the figure as an image without displaying the figure
    fig_invisible = figure('visible', 'off'); % Create an invisible figure
    imshow(overlay_image, []);
    hold on;

    % Plot the reduced all surfaces
    line(reduced_all_surfaces(:,2), reduced_all_surfaces(:,1), 'color', 'r', 'marker', 'o');

    % Draw the line representing the "Top Layer"
    line(top_layer_points(:,2), top_layer_points(:,1), 'color', 'g', 'LineWidth', 2);

    % Plot the shortest distances from depth_points to the Top Layer line
    numDepthPoints = size(depth_points, 1); % Compute once outside lo
    for i = 1:numDepthPoints
        line([depth_points(i,2), x_intersect(i)], [depth_points(i,1), y_intersect(i)], 'color', 'y', 'LineWidth', 1);
    end

    % Set title
    title('Ablated Surfaces (Red), Top Layer (Green), and Vertical Distances (Yellow) on Depth Image');

    % Keep the hold off
    hold off;
    
    % Save the figure as an image
    fig = gcf; % Get current figure handle
    final_image_filename = 'final_image.png'; % Define filename for final image
    saveas(fig, final_image_filename); % Save figure as png image
    close(fig); % Close figure window
    
    % Read the final image
    final_image = imread(final_image_filename);
end