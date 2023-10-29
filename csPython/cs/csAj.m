function [reconstructed, t] = csAj(A_2dMask, y)
    % Get the dimensions of A_2dMask
    [image_rows, image_cols] = size(A_2dMask);

    A_2dMaskVector = A_2dMask(:);  % Flatten the 2D mask to a vector
    A_LinearIdx = find(A_2dMaskVector == 1);  % Find indices where value is 1
    A = [];
    for i = 1:length(A_LinearIdx)
        pixelLocation = A_LinearIdx(i);
        row = zeros(size(A_2dMaskVector, 1), 1);
        row(pixelLocation) = 1;
        row = sparse(row);
        A = [A, row];
    end
    A = A';

    %% Run TVAL3
    clear opts
    opts.mu = 2^8;
    opts.beta = 2^6;
    opts.tol = 1E-4;
    opts.maxit = 300;
    opts.TVnorm = 1;
    opts.nonneg = true;
 
    tic;
    [reconstructed, ] = TVAL3(A,y,image_rows,image_cols,opts);
    t = toc;
end
