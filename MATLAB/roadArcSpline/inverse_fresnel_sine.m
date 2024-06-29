function x_approx = inverse_fresnel_sine(y)
    % Define the function whose root corresponds to the inverse of S(x)
    inverse_S = @(x) fresnels(x) - y;
    
    % Initial guess for the root (starting point for Newton's method)
    x0 = 0;
    
    % Set options for the fzero function (tolerance, maximum iterations, etc.)
    options = optimset('TolX', 1e-10);
    
    % Use fzero to find the root (inverse of S(x)) for the given y
    x_approx = fzero(inverse_S, x0, options);
end
