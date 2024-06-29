function [clothoidArray] = fitArcSpline(init_pos,init_tan,init_curv,clothoid_GT)
% sigma = 0.01; %This is the slope of the curvature plot

theta0 = -(clothoid_GT.init_tan - init_tan);
k0 = -(clothoid_GT.init_curv - init_curv);
sigma = k0 * k0 / (2 * theta0);
xyPairs = [clothoid_GT.allX' clothoid_GT.allY'];

%% First rectify heading and curvature
if(theta0 > 0)
    if(k0 < 0) % init curv is negative
        L = -2 * theta0 / k0;
        k1 = init_curv + sigma*L; % intermediate curvature
        clothoidArray(1) = clothoid_v2(init_pos, init_tan, init_curv, k1, L);
        x0 = clothoidArray(1).allX(end);
        y0 = clothoidArray(1).allY(end);
        angle = clothoidArray(1).final_tan + pi/2;
        
        [closest_point, idx] = findClosestPointOnLine(x0, y0, angle, xyPairs);

        
        realRoadHeading = clothoid_GT.allTangent(idx);
        realRoadCurvature = clothoid_GT.allCurvature(idx);

        headingError = rad2deg(clothoidArray(1).final_tan - realRoadHeading)
        verification = theta0 + k0 * L + sigma * L * L / 2
        curvatureError = realRoadCurvature - clothoidArray(1).final_curv
    elseif(k0 > 0)
        l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
        l2 = l1 - k0/sigma; 
        L = l1 + l2;

        k1 = k0 - sigma*l1;  % intermediate curvature
        k2 = k1 + sigma*l2; % % final curvature

        clothoidArray(1) = clothoid_v2(init_pos, init_tan, init_curv, k1, l1);

        x1 = clothoidArray(1).allX(end);
        y1 = clothoidArray(1).allY(end);
        angle1 = clothoidArray(1).final_tan + pi/2;
        [closest_point1, idx1] = findClosestPointOnLine(x1, y1, angle1, xyPairs);
        realRoadHeading = clothoid_GT.allTangent(idx);
        realRoadCurvature = clothoid_GT.allCurvature(idx);
        headingError = rad2deg(clothoidArray(1).final_tan - realRoadHeading)
        verification = theta0 + k0 * L + sigma * L * L / 2
        curvatureError = realRoadCurvature - clothoidArray(1).final_curv

        intermediate_pos = [clothoidArray(1).allX(end) clothoidArray(1).allY(end)];
        intermediate_tan = clothoidArray(1).final_tan;
        intermediate_curv = clothoidArray(1).final_curv;
        clothoidArray(2) = clothoid_v2(...
            intermediate_pos, intermediate_tan, intermediate_curv, k2, l2);

        x2 = clothoidArray(2).allX(end);
        y2 = clothoidArray(2).allY(end);
        angle2 = clothoidArray(2).final_tan + pi/2;
        [closest_point2, idx2] = findClosestPointOnLine(x2, y2, angle2, xyPairs);
        realRoadHeading = clothoid_GT.allTangent(idx2);
        realRoadCurvature = clothoid_GT.allCurvature(idx2);
        headingError = rad2deg(clothoidArray(2).final_tan - realRoadHeading)
        % verification = theta0 + k0 * L + sigma * L * L / 2
        curvatureError = realRoadCurvature - clothoidArray(2).final_curv        
        % this is not finished
    end
else


end

figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
hold on
axis equal
clothoidArray(1).plotPlain();
% second_clothoid.plotPlain();
legend();


second_clothoid = [];
clothoidArray = [clothoidArray(1) second_clothoid];
% if(deltaHeading > 0)
%     normallyHeadingChange = (k1 + k3) * L / 2;
%     if( (normallyHeadingChange-deltaHeading) < 0) % go above the normal line
%         k2 = (2*sigma*deltaHeading + k1*k1 - k3*L*sigma - k3*k1) / (L*sigma + k1 - k3);
%         clothoidArray(1)_length = (k2 - k1) / sigma;
%     else % go under the normal line
%         k2 = (2*sigma*deltaHeading - k1*k1 - k3*L*sigma + k3*k1) / (L*sigma - k1 + k3);
%         clothoidArray(1)_length = (k1 - k2) / sigma;
%     end
% else
% 
%     l1 = k1/sigma;
%     k2 = 2*deltaHeading * (l1 - targetLength) / (targetLength* ( 2*l1 - targetLength));
% 
%     l2 = -k2/sigma;
% 
%     secondClothoidSlope = (k3 - k2) / (targetLength - l1 - l2);
% 
%     l3 = - k2 / secondClothoidSlope;
%     l4 = targetLength - l1 - l2 -l3;
%     clothoidArray(1)_length = l1 + l2;
% end
% 
% 
% second_clothoid_length = targetLength - clothoidArray(1)_length;
% 
% figure;
% plot([0 clothoidArray(1)_length targetLength],[k1 k2 k3])
% title("Curvature Plot")
% ylabel("m^-1")
% xlabel("Curve length (m)")
% 
% clothoidArray(1) = clothoid(init_pos, init_tan, init_curv, k2, clothoidArray(1)_length, 5, arcSegClass);
% next_pos = [clothoidArray(1).allX(end) clothoidArray(1).allY(end)];
% next_tan = clothoidArray(1).final_tangent;
% next_curv = clothoidArray(1).final_curv;
% 
% second_clothoid = clothoid(next_pos, next_tan, next_curv, targetCurvature, second_clothoid_length, 5, arcSegClass);
% 
% figure;
% plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
% hold on
% axis equal
% clothoidArray(1).plotPlain();
% second_clothoid.plotPlain();
% legend();
% 
% clothoidArray = [clothoidArray(1) second_clothoid];

% optimized = false;
% if( (targetCurvature>0) && (init_curv>0) && ((deltaHeading)<0))
%     %COUNTER MANEUVER IS NEEDED
% 
%     while((~optimized) && (idx~=0) )
%         firstLength = abs(init_curv) / sigma;
%         secondLength = targetLength - targetCurvature*firstLength/init_curv;
% 
%         headingIntegration = init_curv*firstLength/2 - (secondLength - firstLength)^2*init_curv / 4 / firstLength + ...
%             targetCurvature^2*firstLength/2/init_curv;
% 
%         if ( abs(deltaHeading-headingIntegration)  < abs(0.1 * deltaHeading))
%             optimized = true;
%             intermediate_curvature = -(secondLength - firstLength) * init_curv / 2 / firstLength;
% 
%             first_part = clothoid(init_pos,init_tan,init_curv,intermediate_curvature,...
%                 firstLength,cloth_order,arcSegClass);
% 
%             second_part = clothoid([first_part.allX(end) first_part.allY(end)],first_part.final_tangent,...
%                 intermediate_curvature,targetCurvature, secondLength,cloth_order,arcSegClass);
% 
%             clothoidArray = [first_part, second_part];
%         else
%             idx = floor(idx*3/4);
%             targetHeading = clothoid_GT.allTangent(idx);
%             targetCurvature = clothoid_GT.allCurvature(idx);
%             targetLength = idx / 100;
%             deltaHeading = targetHeading - init_tan;
%         end
%     end
% 
% elseif( (targetCurvature>0) && (init_curv>0) && (deltaHeading>0) )
%     while((~optimized) && (idx~=0) )
%         rectificationLength = 2 * deltaHeading / (targetCurvature + init_curv);
%         if ( (abs(targetLength-rectificationLength) / targetLength ) < 0.05)
%             optimized = true;
%             tempClothoid = clothoid(init_pos,init_tan,init_curv,targetCurvature,rectificationLength...
%                 ,cloth_order,arcSegClass);
%             clothoidArray = tempClothoid;
%         else
%             idx = floor(idx*3/4);
%             targetHeading = clothoid_GT.allTangent(idx);
%             targetCurvature = clothoid_GT.allCurvature(idx);
%             targetLength = idx / 100;
%             deltaHeading = targetHeading - init_tan;
%         end
%     end
%     % clothoid(init_pos,init_tan, init_curvature, final_curvature,...
%     %            length,order,arcSegClass)
% 
% end
% 
% %% Account for the position error
% last_pt = [clothoidArray(end).allX(end) clothoidArray(end).allY(end) ];
% last_heading = clothoidArray(end).final_tangent;
% last_curv = clothoidArray(end).final_curv;
% 
% deltaY_vector = [clothoid_GT.allX(idx) clothoid_GT.allY(idx) 0] -  [last_pt 0];
% deltaY = norm(deltaY_vector);
% 
% HEADING_ERROR = rad2deg(last_heading - clothoid_GT.final_tan)
% CURVATURE_END  = last_curv
% CURVATURE_OF_REAL_ROAD = clothoid_GT.allCurvature(idx)
% 
% left_length = (length(clothoid_GT.allX) - idx) / 100; %for each centimeter.(this will be wrong)
% arcLen = left_length / 10;
% 
% radius = abs(1/last_curv);
% 
% center_x = last_pt(1) - radius * sin(last_heading + sign(last_curv) * pi/2); % x-coordinate of the center
% center_y = last_pt(2) + radius * cos(last_heading + sign(last_curv) * pi/2); % y-coordinate of the center
% to_center_vector = [last_pt 0] - [center_x center_y 0];
% 
% angle_rad = atan2(norm(cross(to_center_vector, deltaY_vector)), dot(to_center_vector, deltaY_vector));
% 
% if angle_rad > pi
%     angle_rad = 2*pi - angle_rad;
% end
% 
% radial_effect = angle_rad - pi/2;
% 
% arcs(10) = arcSegment;
% keepDeltaY = zeros(1,10);
% for i = 1:10
%     keepDeltaY(i) = deltaY;
%     radius = radius - sign(radial_effect) * deltaY * 100000;
% 
%     tempArc = arcSegment(last_pt,last_heading, radius, arcLen,sign(last_curv));
%     tempArc.getXY();
%     last_pt = [tempArc.x_coor(end) tempArc.y_coor(end)];
%     last_heading = tempArc.final_angle;
%     GT_idx = ceil(i*arcLen*100 + idx);
%     GT_coor = [clothoid_GT.allX(GT_idx) clothoid_GT.allY(GT_idx)];
%     deltaY = norm(GT_coor - last_pt);
% 
%     arcs(i) = tempArc;
% end
% 
% 
% figure;
% plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
% hold on
% axis equal
% for i = 1:numel(clothoidArray)
%     plot(clothoidArray(i).allX,clothoidArray(i).allY,'Color',[0 0 i/numel(clothoidArray)])
% end
% 
% for j = 1:numel(arcs)
%     plot(arcs(j).x_coor,arcs(j).y_coor,'Color',[1 0 0])
% 
% end
% legend();

end

