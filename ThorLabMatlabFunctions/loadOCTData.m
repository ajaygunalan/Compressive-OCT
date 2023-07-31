function data = loadOCTData(folders, filenames)
    % Initialize structure to hold OCT data
    data = struct('frame', {}, 'imagePath', {});

    for i = 1:numel(filenames)
        folderPath = folders{i};
        fileName = filenames{i};
        fullPath = fullfile(folderPath, fileName);

        % Load the OCT Data
        handle = OCTFileOpen(fullPath);
        intensity = OCTFileGetIntensity(handle);

        % Load depth image
        frame = intensity;
        
        % Save the frame image
        figure('Visible', 'off'); % Explicitly create an invisible figure
        imagesc(frame);
        colormap gray;
        axis off;
        set(gca,'Position',[0 0 1 1]);
        imageName = sprintf('%s.png', fileName);
        imagePath = fullfile(folderPath, imageName);
        saveas(gcf, imagePath);
        close(gcf);
        
        % Save the frame and imagePath into the structure
        data(i).frame = frame;
        data(i).imagePath = imagePath;
    end
end
