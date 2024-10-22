function quinticBezier(b0,b1,b2,b3,b4,b5,N)
    t = unique([0:1/N:1 1]); 
    B1 = b0(1).*(1-t).^5 + ...
        5*b1(1).*t.*(1-t).^4 + ...
        10*b2(1).*t.^2.*(1-t).^3 + ...
        10*b3(1)*t.^3.*(1-t).^2 + ...
        5*b4(1)*t.^4.*(1-t) + ...
        b5(1)*t.^5;
    B2 = b0(2).*(1-t).^5 + ...
        5*b1(2).*t.*(1-t).^4 + ...
        10*b2(2).*t.^2.*(1-t).^3 + ...
        10*b3(2)*t.^3.*(1-t).^2 + ...
        5*b4(2)*t.^4.*(1-t) + ...
        b5(2)*t.^5;
    plot(B1,B2)
    hold all;
    scatter([b0(1) b1(1) b2(1) b3(1) b4(1) b5(1)],[b0(2) b1(2) b2(2) b3(2) b4(2) b5(2)])
