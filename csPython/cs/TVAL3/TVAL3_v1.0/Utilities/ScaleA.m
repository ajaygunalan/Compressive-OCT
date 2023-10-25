% This function scales the variables mu, A, and b so that the largest eigenvalue of the matrix A'*A is 1. 
% This scaling is performed to ensure that the new problem obtained after scaling is equivalent to the old problem. 
% If the option input is assigned, mu will be scaled accordingly.
% 
% The function uses the eigs function to compute the largest eigenvalue of A'*A using the lm (largest magnitude) option. 
% If the largest eigenvalue is greater than 1, the function scales mu, A, and b.

function [mu,A,b] = ScaleA(n,mu,A,b,option)

% Scales mu, A and f so that the largest eigenvalue of A'*A is 1 and the
% new problem
%
% min sum_i (||wi|| + beta/2 ||Diu - wi||^2) + mu/2 ||Au - b||^2
%
% is equivalent to the old one.  
%
% If option is assigned, mu will be scaled accordingly.
%
% Written by: Chengbo Li

eopts.disp = 0;
eopts.tol = .05;
if ~isreal(A(rand(n,1),1))
    eopts.isreal = false;
end

fh = @(x) A(A(x,1),2);
s2 = eigs(fh,n,1,'lm',eopts);
if real(s2) > 1 + 1e-10
    if option
        mu = mu*s2;
    end
    b = b/sqrt(s2);
    A = @(x,mode) A(x,mode)/sqrt(s2);
end

return