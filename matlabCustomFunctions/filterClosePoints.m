function [filteredPoints] = filterClosePoints(points, minDistance)
    n = size(points, 1);
    % Initialize filteredPoints with the first point
    filteredPoints = points(1, :);
    % For each point
    for i = 2:n
        % Calculate the distance to the existing points in filteredPoints
        distances = pdist2(points(i, :), filteredPoints);
        % If the point is further away than minDistance from all existing points, add it
        if all(distances > minDistance)
            filteredPoints = [filteredPoints; points(i, :)];
        end
    end
end
