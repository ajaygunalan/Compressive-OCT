function [A, samplerMatrix, samplerLinearIdx, CR] = DesignSamplingMatrix(image_rows, image_cols)
    xLen = image_rows*image_cols;
  
    %% CASE 1: Random Mask
    % CompressionRatio = 0.3;
    % measurementLen = round(image_rows*image_cols*CompressionRatio);
    % 
    % % Random Sampling Matrix
    % samplerMatrix = zeros(image_rows, image_cols);
    % 
    % % no. of points to be measured on the image
    % samplerMatrix(1:measurementLen) = 1; 
    % 
    % % shuffle the matrix
    % samplerMatrix(randperm(numel(samplerMatrix))) = samplerMatrix;
    % 
    % % Sampler Matrix is Ready
    % samplerMatrix = logical(samplerMatrix);
    %% CASE 2: Skipping alternate lines
    samplerMatrix = ones(image_rows, image_cols);
    imageSize = 1:image_rows;
    image_half = imageSize (rem(imageSize,2)==0);
    for i = image_half
        samplerMatrix(i,:) = 0;
    end
    for i = image_half
        samplerMatrix(:,i) = 0;
    end
    samplerMatrix = logical(samplerMatrix);
    %% CASE 3: Bibin Style
    % samplerMatrix = rand(image_rows, image_cols) < CompressionRatio;
    %% CASE 4: Bright Field Image
    measurementLen = sum(samplerMatrix(:) == 1);
    %% Get measurements
    samplerV = reshape(samplerMatrix,[],1);
    samplerLinearIdx = find(samplerV==1);

%     x = sample(:);
%     y = x(samplerLinearIdx);
%     measurements = samplerMatrix.*sample;

%     measurements = reshape(measurements,[],1);
%     measurements = nonzeros(measurements);
%     
%     mask = zeros(size(x));
%     mask(samplerLinearIdx) = y ;
    %% y = A*x; Now, what is A?
    samplerV = reshape(samplerMatrix,[],1);
    samplerLinearIdx = find(samplerV==1);
    A = [];
    for i=1:measurementLen
        pixelLocation = samplerLinearIdx(i);
        row =  zeros(xLen, 1);
        row(pixelLocation) = 1;
        row = sparse(row);
        A = [A, row];
    end
    A = A';
    CR = (measurementLen/xLen*100);
end