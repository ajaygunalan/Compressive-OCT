function result = depthEstimation(after)
    % Extracting edges
    topEdge = after(1:15, :); % Top 15 rows
    bottomEdge = after(end-14:end, :); % Bottom 15 rows
    leftEdge = after(:, 1:30); % Left 30 columns
    rightEdge = after(:, end-29:end); % Right 30 columns
    
    % Calculating medians for each edge
    medianTop = median(topEdge, 'all');
    medianBottom = median(bottomEdge, 'all');
    medianLeft = median(leftEdge, 'all');
    medianRight = median(rightEdge, 'all');
    
    % Combine all medians into a vector
    allMedians = [medianTop, medianBottom, medianLeft, medianRight];
    
    % Calculating the overall median
    surface = median(allMedians);


    % Define the ROI boundaries for center
    roi_rows = 125:175;    
    roi_cols = 250:350; 
    roi_difference = after(roi_rows, roi_cols);
    deep = median(roi_difference(:));
    result = deep - surface;
end


%     % Check if the input matrices have the same size
%     if size(before) ~= size(after)
%         error('The input matrices must be of the same size.');
%     end
%     difference = after -before;
% 
%     % Save as a figure with detailed colorbar settings
%     fig3 = figure('Visible', 'off'); 
%     imagesc(difference);
%     pbaspect([1 1 1])
%     axis tight;
%     minData = min(difference(:));
%     maxData = max(difference(:));
%     cb3 = colorbar; 
%     set(cb3, 'Limits', [minData, maxData]);
%     T3 = linspace(minData, maxData, 5); 
%     set(cb3, 'Ticks', T3); 
%     ylabel(cb3, 'mm');
%     TL3 = arrayfun(@(x) sprintf('%.2f', x), T3, 'UniformOutput', false); 
%     set(cb3, 'TickLabels', TL3);
%     % Save the figure as a MATLAB figure file
%     imageFilename3 = [prefix, 'depthMap.png'];
%     fullImagePath3 = fullfile(folderLocation, imageFilename3); 
%     saveas(fig3, fullImagePath3, 'png');
%     imagesc(difference);
% 
%     % Save data as a text file
%     textFilename = [prefix, 'depthMap.csv'];
%     fullImagePath3 = fullfile(folderLocation, textFilename);
%     writematrix(difference, fullImagePath3);

