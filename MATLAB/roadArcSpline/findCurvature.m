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
        disp(['Points: ',num2str(pts(i,:)) ])
        disp(['Curvature: ',num2str(curvature(i - 1)*1000,3) ])
%         Ax = pts(i,1); 
%         Ay = pts(i,2);
%         Bx = pts(i-1,1); 
%         By = pts(i-1,2); 
%         Cx = pts(i+1,1); 
%         Cy = pts(i+1,2); 
%         figure; 
%         hold all; 
% %         quiver(Bx,By,Ax,Ay); 
% %         quiver(Ax,Ay,Cx,Cy); 
% %         quiver(Ax,Ay,e(1),e(2)); 
% %         quiver(Ax,Ay,f(1),f(2)); 
%         plot([Bx,Ax],[By,Ay],'Color',[0 1 0]); 
%         plot([Ax,Cx],[Ay,Cy],'Color',[0 0.5 0]); 
%         plot([Ax,Ax+e(1)],[Ay,Ay+e(2)],'Color',[0 0 1]); 
%         plot([Ax,Ax+f(1)],[Ay,Ay+f(2)],'Color',[1 0 0]);
        
    end

end 