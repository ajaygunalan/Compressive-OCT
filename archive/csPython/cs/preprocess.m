function image = preprocess(image, image_rows, image_cols)
    image = imresize(image, [image_rows, image_cols]);
    image = double(im2gray(image));
    image = image./max(max(image)); 
return;