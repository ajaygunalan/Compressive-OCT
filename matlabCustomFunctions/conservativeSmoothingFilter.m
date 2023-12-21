function y = conservativeSmoothingFilter(x, neighborhoodSize)
    % x is the input image
    % neighborhoodSize is the size of the neighborhood for the conservative smoothing filter
    % y will be the output image with conservative smoothing applied
    
    [rows, cols] = size(x);  % Get the size of the input image
    y = x;  % Initialize the output image
    halfSize = floor(neighborhoodSize / 2);  % Calculate half the size of the neighborhood
    
    % Loop over the image, skipping the borders according to the neighborhood size
    for i = (1 + halfSize):(rows - halfSize)
        for j = (1 + halfSize):(cols - halfSize)
            % Extract the neighborhood around the current pixel
            neighborhood = x((i - halfSize):(i + halfSize), (j - halfSize):(j + halfSize));
            
            % Find the maximum and minimum values in the neighborhood
            max_val = max(neighborhood(:));
            min_val = min(neighborhood(:));
            
            % Get the value of the current pixel
            current_pixel = x(i, j);
            
            % Apply the conservative smoothing filter logic
            if current_pixel > max_val
                y(i, j) = max_val;
            elseif current_pixel < min_val
                y(i, j) = min_val;
            end
        end
    end
end