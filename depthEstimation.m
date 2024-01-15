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

    % Plotting the histogram of the difference
    figure; % Creates a new figure window
    histogram(difference);
    xlabel('Difference Value');
    ylabel('Frequency');
    title('Histogram of Depth Differences');

    % Optionally, you can save the histogram figure as well
    histogramFilename = [prefix, 'depthHistogram.png'];
    fullHistogramPath = fullfile(folderLocation, histogramFilename);
    saveas(gcf, fullHistogramPath, 'png');
    result = round(max(difference(:)), 2);
end
