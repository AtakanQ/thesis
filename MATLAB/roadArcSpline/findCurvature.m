function [curvature, center] = findCurvature(pts)

curvature = zeros(length(pts)-2,1);
center = zeros(length(pts)-2,2);

    for i = 2: (length(pts) - 1)
        %Calculate AB
        AB = pts(i - 1,:) - pts(i,:);
        AB = [AB 0];
        normAB = norm( AB );
        
        %Calculate AB
        AC = pts(i + 1,:) - pts(i,:); 
        AC = [AC 0]; 
        normAC = norm( AC );
        
        %Calculate angle
        % small_angle = atan2(norm(cross(AB,AC)),dot(AB,AC));
    
        %Calculate D
        D = cross(AB,AC);
        norm_D = norm(D);
        % norm_D = normAB * normAC * sin(small_angle);
        % D = [0 0 norm_D];
    
        %Calculate D
        E = cross(D,AB);
        norm_E = norm(E);
    
        %Calculate F
        F = cross(D,AC);
        norm_F = norm(F);
    
        e = E / norm_E;
        f = F / norm_F;
    
        rho = ( normAC*normAC * E - normAB*normAB * F) / (2 * norm_D * norm_D);
        
        crossProduct = AC(1) * rho(2) - AC(2) * rho(1);


        curvature(i - 1) = sign(crossProduct) * 1/norm(rho);
        center(i - 1,:) = pts(i,:) + rho(1:2);
        % disp(['Points: ',num2str(pts(i,:)) ])
        % disp(['Curvature: ',num2str(curvature(i - 1)*1000,3) ])
        
    end

end 