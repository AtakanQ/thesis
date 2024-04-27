function [clothoidArray] = fitArcSpline(init_pos,init_tan,init_curv,clothoid_GT)
sigma = 0.01; %This is the slope of the curvature plot
idx = floor(length(clothoid_GT.allTangent)*1/2);
targetHeading = clothoid_GT.allTangent(idx);
targetCurvature = clothoid_GT.allCurvature(idx);
targetLength = idx / 100;
% targetLength = targetLength*11/10
arcSegClass = arcSegment;
cloth_order = 5;
deltaHeading = targetHeading - init_tan;
%% First rectify heading and curvature
k1 = init_curv;
k3 = targetCurvature;
L = targetLength;

k2 = [];
if(deltaHeading > 0)
    normallyHeadingChange = (k1 + k3) * L / 2;
    if( (normallyHeadingChange-deltaHeading) < 0) % go above the normal line
        k2 = (2*sigma*deltaHeading + k1*k1 - k3*L*sigma - k3*k1) / (L*sigma + k1 - k3);
        first_clothoid_length = (k2 - k1) / sigma;
    else % go under the normal line
        k2 = (2*sigma*deltaHeading - k1*k1 - k3*L*sigma + k3*k1) / (L*sigma - k1 + k3);
        first_clothoid_length = (k1 - k2) / sigma;
    end
else

    l1 = k1/sigma;
    k2 = 2*deltaHeading * (l1 - targetLength) / (targetLength* ( 2*l1 - targetLength));

    l2 = -k2/sigma;
   
    secondClothoidSlope = (k3 - k2) / (targetLength - l1 - l2);

    l3 = - k2 / secondClothoidSlope;
    l4 = targetLength - l1 - l2 -l3;
    first_clothoid_length = l1 + l2;
end


second_clothoid_length = targetLength - first_clothoid_length;

figure;
plot([0 first_clothoid_length targetLength],[k1 k2 k3])
title("Curvature Plot")
ylabel("m^-1")
xlabel("Curve length (m)")

first_clothoid = clothoid(init_pos, init_tan, init_curv, k2, first_clothoid_length, 5, arcSegClass);
next_pos = [first_clothoid.allX(end) first_clothoid.allY(end)];
next_tan = first_clothoid.final_tangent;
next_curv = first_clothoid.final_curv;

second_clothoid = clothoid(next_pos, next_tan, next_curv, targetCurvature, second_clothoid_length, 5, arcSegClass);

figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
hold on
axis equal
first_clothoid.plotPlain();
second_clothoid.plotPlain();
legend();

clothoidArray = [first_clothoid second_clothoid];

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

