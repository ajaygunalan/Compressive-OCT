% Nested function to rotate and display the image
function rotateImage(fig)
    % Retrieve the image data, handle, and sliders from guidata
    handles = guidata(fig);
    depthWYS = handles.depthWYS;
    imgHandle = handles.imgHandle;
    sliderCoarse = handles.sliderCoarse;
    sliderFine = handles.sliderFine;

    % Get the values from both sliders
    angleCoarse = get(sliderCoarse, 'Value');
    angleFine = get(sliderFine, 'Value');

    % Combine the values of the two sliders to determine the rotation angle
    angle = angleCoarse + angleFine;

    % Rotate the image and update the display
    rotatedImage = imrotate(depthWYS, angle, 'bicubic', 'crop');
    set(imgHandle, 'CData', rotatedImage);
    title(['Rotation Angle: ', num2str(angle)]);
end