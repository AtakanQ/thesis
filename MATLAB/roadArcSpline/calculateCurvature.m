function kappa = calculateCurvature(x, y)

    dx = gradient(x);
    dy = gradient(y);
    

    ddx = gradient(dx);
    ddy = gradient(dy);
    

    kappa = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);
    

    kappa(isnan(kappa) | isinf(kappa)) = 0;
end