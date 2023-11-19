function [x, y, curvature] = generateBezier(P0,P1,P2,P3,P4,P5)

    t = 0:0.001:1;
x = (1-t).^5 * P0(1) + 5*t.*(1-t).^4 * P1(1) + 10*t.^2.*(1-t).^3 * P2(1) + ...
    10*t.^3.*(1-t).^2 * P3(1) + 5*t.^4.*(1-t)*P4(1) + t.^5 * P5(1);

y = (1-t).^5 * P0(2) + 5*t.*(1-t).^4 * P1(2) + 10*t.^2.*(1-t).^3 * P2(2) + ...
    10*t.^3.*(1-t).^2 * P3(2) + 5*t.^4.*(1-t)*P4(2) + t.^5 * P5(2);

% Calculate first derivative
dx_dt = -5*(1-t).^4*P0(1) + 5*(1-t).^4*P1(1) - 20*t.*(1-t).^3*P1(1) + ...
        20*t.*(1-t).^3*P2(1) + 30*t.^2.*(1-t).^2*P2(1) - 30*t.^2.*(1-t).^2*P3(1) + ...
        10*t.^3.*(1-t)*P3(1) - 10*t.^3.*(1-t)*P4(1) - t.^4*P4(1) + t.^4*P5(1);
    
dy_dt = -5*(1-t).^4*P0(2) + 5*(1-t).^4*P1(2) - 20*t.*(1-t).^3*P1(2) + ...
        20*t.*(1-t).^3*P2(2) + 30*t.^2.*(1-t).^2*P2(2) - 30*t.^2.*(1-t).^2*P3(2) + ...
        10*t.^3.*(1-t)*P3(2) - 10*t.^3.*(1-t)*P4(2) - t.^4*P4(2) + t.^4*P5(2);

% Calculate second derivative
d2x_dt2 = 20*(1-t).^3*P1(1) - 60*(1-t).^2*P2(1) + 60*(1-t).*t.*P3(1) - ...
          20*t.^3*P4(1) + 20*(1-t).^2*P1(1) - 120*(1-t).*t.*P2(1) + ...
          60*t.^2.*P3(1) + 20*(1-t).*t.*P2(1) - 60*t.^2.*P3(1) + 20*t.^3*P4(1);

d2y_dt2 = 20*(1-t).^3*P1(2) - 60*(1-t).^2*P2(2) + 60*(1-t).*t.*P3(2) - ...
          20*t.^3*P4(2) + 20*(1-t).^2*P1(2) - 120*(1-t).*t.*P2(2) + ...
          60*t.^2.*P3(2) + 20*(1-t).*t.*P2(2) - 60*t.^2.*P3(2) + 20*t.^3*P4(2);

% Calculate curvature
numerator = abs(dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2);
denominator = (dx_dt.^2 + dy_dt.^2).^(3/2);
curvature = numerator ./ denominator;

% Determine the sign of curvature (right or left turn)
cross_product = dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2;
curvature = curvature .* sign(cross_product);


end