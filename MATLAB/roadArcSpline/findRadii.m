function radii = findRadii(pts)

radii = zeros(length(pts)-2,1);

plot(pts(:,1), pts(:,2))
hold on
plot(pts(:,1), pts(:,2),'*')

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
        small_angle = atan2(norm(cross(AB,AC)),dot(AB,AC));
    
        %Calculate D
        norm_D = normAB * normAC * sin(small_angle);
        D = [0 0 norm_D];
    
        %Calculate D
        E = cross(D,AB);
        % norm_E = norm(E);
    
        %Calculate F
        F = cross(D,AC);
        % norm_F = norm(F);
    
        % e = E / norm_E;
        % f = F / norm_F;
    
        rho = ( normAC*normAC * E - normAB*normAB * F) / (2 * norm_D * norm_D);
        radii(i - 1) = 1/norm(rho);
        
    end

end 