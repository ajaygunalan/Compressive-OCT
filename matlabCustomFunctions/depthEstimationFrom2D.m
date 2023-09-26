function [depth_points, perpendicular_distances] = depthEstimationFrom2D(folderPath, filename)
    %% 1. Detect the ablate surface -> Binary, Dilate, Erode, edge, Contnour. 
    fullPath = fullfile(folderPath, filename);
    % Separate the filename into name and extension
    [~, name, ext] = fileparts(filename);

    % Create new filename for the middle image
    middle_image_name = strcat(name, 'middle', ext);
    middle_image_filename = fullfile(folderPath, middle_image_name);

    % Create new filename for the final image
    final_image_name = strcat(name, 'final', ext);
    final_image_filename = fullfile(folderPath, final_image_name);

    % Create new filename for the corresponding text file
    txt_name = strcat(name, 'middle', '.txt');
    txt_filename = fullfile(folderPath, txt_name);

    depthI = imread(fullPath);

    % Check if the image is RGB
    if ndims(depthI) == 3
        % Convert RGB to grayscale
        depthI = rgb2gray(depthI);
    end

    [img_height, img_width, ] = size(depthI);

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

    % remove close points
    minDistance = 3.6; % adjust this as needed
    for idx = 1:ablate_surface_idx
        ablate_surface{idx} = filterClosePoints(ablate_surface{idx}, minDistance);
    end

    ablate_surface_combined = [];
    % Combine multiple ablate surfaces if they exist
    for idx = 1:length(ablate_surface)
        ablate_surface_combined = [ablate_surface_combined; ablate_surface{idx}];
    end

    % Visualize the image with reduced contours
    figure;
    imshow(frame_contours_with_reduced, []);
    hold on;

    % Draw each ablate surface
    for idx = 1:ablate_surface_idx
        line(ablate_surface{idx}(:,2), ablate_surface{idx}(:,1), 'color','r','linewidth',1.5,'marker','o','markersize',5);
    
        % Annotate the surface with surface_idx value
        mid_point = round(size(ablate_surface{idx}, 1)/2);
        text(ablate_surface{idx}(mid_point, 2), ablate_surface{idx}(mid_point, 1), num2str(idx), 'Color', 'yellow', 'FontSize', 14);
    end
    title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image');

       
    % Plot the ablate_surface points for selection
    for idx = 1:length(ablate_surface_combined)
        plot(ablate_surface_combined(idx,2), ablate_surface_combined(idx,1), 'mo', 'MarkerSize', 10);
    end

    % Capture only the content within the axes as an image
    ax = gca; % Get handle to current axes
    F = getframe(ax);
    I = F.cdata;
    % Save the captured image
    imwrite(I, middle_image_filename);
    %% 3. Select the ablated contour and final the width and depth
    % Allow the user to draw a bounding box on the existing figure
    h = images.roi.Rectangle(gca, 'Position', [10, 10, 100, 100]); % Initial position
    draw(h); % Make it interactive
    position = h.Position; % [xmin ymin width height]
    
    % Extract the coordinates of the bounding box
    xmin = position(1);
    ymin = position(2);
    xmax = xmin + position(3);
    ymax = ymin + position(4);

    % Normalize the coordinates and dimensions
    center_x = (xmin + xmax) / 2 / img_width;
    center_y = (ymin + ymax) / 2 / img_height;
    width = (xmax - xmin) / img_width;
    height = (ymax - ymin) / img_height;

    class_id = 1;  % Replace with the actual class ID if needed
    fileID = fopen(txt_filename, 'w');
    fprintf(fileID, '%d %f %f %f %f\n', class_id, center_x, center_y, width, height);
    fclose(fileID);
  
    % Filter the contour points that are within the bounding box
    idx = ablate_surface_combined(:, 2) >= xmin & ablate_surface_combined(:, 2) <= xmax & ...
          ablate_surface_combined(:, 1) >= ymin & ablate_surface_combined(:, 1) <= ymax;
    filtered_points = ablate_surface_combined(idx, :);

    %pointX(2)is the x cordinate
    % Sort by x-coordinate (2nd column) to get leftmost and rightmost points
    sorted_by_x = sortrows(filtered_points, 2);
    point1 = sorted_by_x(1, :);  % Leftmost point
    point2 = sorted_by_x(end, :);  % Rightmost point
    
    % Sort by y-coordinate (1st column) to get the bottommost point
    sorted_by_y = sortrows(filtered_points, 1);
    point3 = sorted_by_y(end, :);  % Bottommost point
    depth_points = [point3(2), point3(1)]; %point3(2) is the X cordinate

    % Calculate the distance between the first two points
    distance = sqrt((point2(2) - point1(2))^2 + (point2(1) - point1(1))^2);
    
    % Find the equation of the line formed by the first two points
    % Equation of line: y = mx + c
    m = (point2(1) - point1(1)) / (point2(2) - point1(2));
    c = point1(1) - m * point1(2);
    
    % Find the equation of the perpendicular line passing through the third point
    % Slope of perpendicular line: -1/m
    m_perpendicular = -1 / m;
    c_perpendicular = point3(1) - m_perpendicular * point3(2);
    
    % Find the intersection point
    x_intersection = (c_perpendicular - c) / (m - m_perpendicular);
    y_intersection = m * x_intersection + c;
    
    % Calculate the length of the perpendicular line
    perpendicular_distances = sqrt((x_intersection - point3(2))^2 + (y_intersection - point3(1))^2);
    perpendicular_distances = perpendicular_distances*0.25/54; % Converted from pixel into mm
    
    % Plot ablate_surface points
    plot(ablate_surface_combined(:, 2), ablate_surface_combined(:, 1), 'mo', 'MarkerSize', 10);
    
    % Plot lines and points
    plot([point1(2), point2(2)], [point1(1), point2(1)], 'g', 'LineWidth', 2); % Green line
    plot([point3(2), x_intersection], [point3(1), y_intersection], 'y', 'LineWidth', 2); % Yellow line
    plot([point1(2), point2(2), point3(2)], [point1(1), point2(1), point3(1)], 'ro', 'MarkerSize', 10); % Red points
    
    hold off;
    % Capture only the content within the axes as an image
    ax = gca; % Get handle to current axes
    F = getframe(ax);
    I = F.cdata;
    % Save the captured image
    imwrite(I, final_image_filename);

    % Close the figure
    close;
end