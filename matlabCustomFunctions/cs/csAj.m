function [reconstructed, t] = csAj(A, y, image_rows, image_cols)
    %% Run TVAL3
    clear opts
    opts.mu = 2^8;
    opts.beta = 2^5;
    opts.tol = 1E-3;
    opts.maxit = 300;
    opts.TVnorm = 1;
    opts.nonneg = true;
 
    tic;
    [reconstructed, ] = TVAL3(A,y,image_rows,image_cols,opts);
    t = toc;
end