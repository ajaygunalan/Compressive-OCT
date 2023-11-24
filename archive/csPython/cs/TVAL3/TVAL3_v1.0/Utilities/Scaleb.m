% The function Scaleb scales the input signal b and the scaling factor mu according to a threshold. If the difference between the maximum and minimum values of b is less than 0.5, then b is scaled up by a factor of 2, and mu is scaled down by the same factor. If the difference is greater than 1.5, then b is scaled down by a factor of 2, and mu is scaled up by the same factor. If the difference is between these two thresholds, no scaling is performed.
% 
% The function takes three input arguments:
% 
% mu: The scaling factor
% b: The input signal
% option: A Boolean value indicating whether to scale mu or not.
% The function returns:
% 
% mu: The scaled scaling factor.
% b: The scaled input signal.
% scl: The scaling factor used.
function [mu,b,scl] = Scaleb(mu,b,option)

% Scales mu and f so that the finite difference of f is neither too small 
% nor too large.
%
% If option is assigned, mu will be scaled accordingly.
%
% Written by: Chengbo Li


threshold1 = .5;      % threshold is chosen by experience.
threshold2 = 1.5;
scl = 1;
b_dif = abs(max(b) - min(b));

if b_dif < threshold1
    scl = threshold1/b_dif;
    b = scl*b;
    if option
        mu = mu/scl;
    end
else if b_dif > threshold2
        scl = threshold2/b_dif;
        b = scl*b;
        if option
            mu = mu/scl;
        end
    end
end

return