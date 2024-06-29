function [b1,b2,b3,b4] = quinticBezier_computation(b0,b5,t1,t2,k1,k2)
    b1 = b0 + 1/5*t1;
    b4 = b5 - 1/5*t2; 
    % Computation for acceleration of initial point
    a1 = t1(1); 
    a2 = t1(2); 
    K1 = k1*norm(t1)^3;
    if a2 ~= 0
        c2 = 1; 
        c1 = (a1-K1)/a2; 
    elseif a1 ~= 0
        c1 = 1; 
        c2 = (K1+a2)/a1; 
    else
        return; 
    end
    ddB0 = [c1; c2]; 
    b2 = ddB0/20 - b0 + 2*b1; 
    % Computation for accelaration of final point
    a1 = t2(1); 
    a2 = t2(2); 
    K2 = k2*norm(t2)^3;
    if a2 ~= 0
        c2 = 1; 
        c1 = (a1-K2)/a2; 
    elseif a1 ~= 0
        c1 = 1; 
        c2 = (K2+a2)/a1; 
    else
        return; 
    end
    ddB1 = [c1; c2]; 
    b3 = ddB1/20 - b5 + 2*b4; 
end