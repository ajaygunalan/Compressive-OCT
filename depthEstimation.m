function result = depthEstimation(after, before,prefix, folderLocation)
    % Check if the input matrices have the same size
    if size(before) ~= size(after)
        error('The input matrices must be of the same size.');
    end
    difference = after -before;

    % Save as a figure with detailed colorbar settings
    fig3 = figure('Visible', 'off'); 
    imagesc(difference);
    pbaspect([1 1 1])
    axis tight;
    minData = min(difference(:));
    maxData = max(difference(:));
    cb3 = colorbar; 
    set(cb3, 'Limits', [minData, maxData]);
    T3 = linspace(minData, maxData, 5); 
    set(cb3, 'Ticks', T3); 
    ylabel(cb3, 'mm');
    TL3 = arrayfun(@(x) sprintf('%.2f', x), T3, 'UniformOutput', false); 
    set(cb3, 'TickLabels', TL3);
    % Save the figure as a MATLAB figure file
    imageFilename3 = [prefix, 'depthMap.png'];
    fullImagePath3 = fullfile(folderLocation, imageFilename3); 
    saveas(fig3, fullImagePath3, 'png');
    imagesc(difference);

    % Save data as a text file
    textFilename = [prefix, 'depthMap.csv'];
    fullImagePath3 = fullfile(folderLocation, textFilename);
    writematrix(difference, fullImagePath3);

    % Define the ROI boundaries
    roi_rows = 125:175;    
    roi_cols = 250:350; 
    roi_difference = difference(roi_rows, roi_cols);
    result = median(roi_difference(:));
end


