function result = depthEstimation(after, before)
    threshold = 0.65;
    % Define the ROI boundaries for the center
    roi_rows = 125:175;    
    roi_cols = 250:350; 

    % Process 'before' image
    beforeROI = before(roi_rows, roi_cols);
    beforeVal = median(beforeROI(:));

    % Process 'after' image
    afterROI = after(roi_rows, roi_cols);
    afterFiltered = afterROI(afterROI >= threshold); 
    afterVal = median(afterFiltered(:));

    % Calculate the result
    result = afterVal - beforeVal;   
end



%     % Extracting edges
%     topEdge = after(1:15, :); % Top 15 rows
%     bottomEdge = after(end-14:end, :); % Bottom 15 rows
%     leftEdge = after(:, 1:30); % Left 30 columns
%     rightEdge = after(:, end-29:end); % Right 30 columns
%     % Apply threshold to each edge
%     topEdge_thresholded = topEdge(topEdge >= threshold);
%     bottomEdge_thresholded = bottomEdge(bottomEdge >= threshold);
%     leftEdge_thresholded = leftEdge(leftEdge >= threshold);
%     rightEdge_thresholded = rightEdge(rightEdge >= threshold);
%     % Calculate medians for each edge using thresholded values
%     medianTop = median(topEdge_thresholded, 'all');
%     medianBottom = median(bottomEdge_thresholded, 'all');
%     medianLeft = median(leftEdge_thresholded, 'all');
%     medianRight = median(rightEdge_thresholded, 'all');
%     % Combine all medians into a vector
%     allMedians = [medianTop, medianBottom, medianLeft, medianRight];
%     % Calculate the overall median
%     surface = median(allMedians);