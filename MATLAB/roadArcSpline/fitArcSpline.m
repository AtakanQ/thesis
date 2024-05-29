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

        positionError = norm(closest_point - [x0 y0]);
        
        final_tangent = clothoidArray(1).final_tan;
        final_tangent_vector = [cos(final_tangent) sin(final_tangent) 0];
        to_closest_point = [closest_point 0] - [x0 y0 0];

        crossproduct = cross(final_tangent_vector,to_closest_point);
        positionError = - sign(crossproduct(3))*...
            norm(closest_point - [x0 y0]);
        
        lengthSoFar = L;

        realRoadHeading = clothoid_GT.allTangent(idx);
        realRoadCurvature = clothoid_GT.allCurvature(idx);

        headingError = rad2deg(clothoidArray(1).final_tan - realRoadHeading)
        verification = theta0 + k0 * L + sigma * L * L / 2
        curvatureError = realRoadCurvature - clothoidArray(1).final_curv
    elseif(k0 > 0)
        l1 = k0/sigma + sqrt(k0^2/2/(sigma^2) + theta0/sigma);  
        l2 = l1 - k0/sigma; 
        L = l1 + l2;

        k1 = init_curv - sigma*l1;  % intermediate curvature
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
        
        final_tangent = clothoidArray(2).final_tan;
        final_tangent_vector = [cos(final_tangent) sin(final_tangent) 0];
        to_closest_point = [closest_point2 0] - [x2 y2 0];
        crossproduct = cross(final_tangent_vector,to_closest_point);
        positionError = - sign(crossproduct(3))*...
            norm(closest_point2 - [x2 y2]);

        realRoadHeading = clothoid_GT.allTangent(idx2);
        realRoadCurvature = clothoid_GT.allCurvature(idx2);
        headingError = rad2deg(clothoidArray(2).final_tan - realRoadHeading)
        % verification = theta0 + k0 * L + sigma * L * L / 2
        curvatureError = realRoadCurvature - clothoidArray(2).final_curv        
        % this is not finished
        lengthSoFar = l1 + l2;
    end
else
    %Symmetric

end

figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
hold on
axis equal
clothoidArray(1).plotPlain();
% second_clothoid.plotPlain();
legend();

%% Compansate for position error
% lengthSoFar
currentPosition = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
currentHeading = clothoidArray(end).final_tan;
initialCurvature = clothoidArray(end).final_curv;


positionCompensationLength = 15; % let this be a constant for now
deltaY = positionError; 
lambda = 0.5; 
gamma = 0.5;
[alpha,k_peak] = computeAlphak1(positionCompensationLength,lambda,gamma,-deltaY,0); 

numClothoids = numel(clothoidArray);
% reach k_peak
clothoidArray(numClothoids + 1) = clothoid_v2(...
    currentPosition, currentHeading, initialCurvature,...
    k_peak + initialCurvature, positionCompensationLength/8);

% update vehicle position
currentPosition = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
currentHeading = clothoidArray(end).final_tan;
currentCurvature = clothoidArray(end).final_curv;


% get back to initialCurvature
clothoidArray(numClothoids + 2) = clothoid_v2(...
    currentPosition, currentHeading, currentCurvature,...
    initialCurvature, positionCompensationLength/8);

% update vehicle position
currentPosition = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
currentHeading = clothoidArray(end).final_tan;
currentCurvature = clothoidArray(end).final_curv;

% keep moving with same curvature (corresponds to line part for bi elementary paths)
middleArc = arcSegment(currentPosition,currentHeading, abs(1/currentCurvature), ...
    positionCompensationLength/2,sign(currentCurvature));
middleArc.getXY()
currentPosition = [middleArc.x_coor(end) middleArc.y_coor(end)];
currentHeading = middleArc.final_angle;

% get to -k_peak
clothoidArray(numClothoids + 3) = clothoid_v2(...
    currentPosition, currentHeading, initialCurvature,...
    -k_peak + initialCurvature, positionCompensationLength/8);

% update vehicle position
currentPosition = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)];
currentHeading = clothoidArray(end).final_tan;
currentCurvature = clothoidArray(end).final_curv;

% get back to initial_curvature
clothoidArray(numClothoids + 4) = clothoid_v2(...
    currentPosition, currentHeading, -k_peak + initialCurvature,...
    initialCurvature, positionCompensationLength/8);

% update vehicle position
currentPosition = [clothoidArray(end).allX(end) clothoidArray(end).allY(end)]
currentHeading = clothoidArray(end).final_tan
currentCurvature = clothoidArray(end).final_curv

figure;
plot(clothoid_GT.allX,clothoid_GT.allY,'Color',[0 1 0],'DisplayName','Ground Truth')
hold on
axis equal
for i = 1:numel(clothoidArray)
    clothoidArray(i).plotPlain([i/10 i/10 0], "Clothoid num:" + num2str(i));
end
plot(middleArc.x_coor,middleArc.y_coor,"DisplayName","Arc","LineWidth",2)
legend();

end
