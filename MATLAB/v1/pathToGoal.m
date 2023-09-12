% close all
clear

%% Path expression
% enter challenging values here
% v = 100 m/s radius = 490meters min_path_length = 55m 
% Emax = 4% fmax = 0.12

radius = 490; %m 
path_length = 98; %m given
theta = path_length / radius;

% initial and final points
init_pos = [0 0];
init_tan = 0; %45 deg
final_pos = [radius*sin(theta) radius*(1-cos(theta))];
final_tan = theta; %45 deg

% init_pos = [19.784661402450800,0.395745995589228];
% init_tan =0.040000000000000; %45 deg
% final_pos = [39.158200044894620,1.567163911716475];
% final_tan = 0.080000000000000; %45 deg
%tolerances
angleTolerance = deg2rad(0); %degrees
deltaYTolerance = 0.1;

% number of total candidates are calculated here
numLenCandidates = 100;
numAngleCandidates = 1;

minLength = norm(init_pos - final_pos);
maxLength = abs(final_pos(1) - init_pos(1)) + abs(final_pos(2) - init_pos(2));
lenCandidates = linspace(minLength,maxLength,numLenCandidates);

angleCandidates = linspace(final_tan - angleTolerance,final_tan + angleTolerance, numAngleCandidates);

validLenIndices = [];
validAngleIndices = [];

deltaY = abs(final_pos(2) - init_pos(2));
for i = 1:numLenCandidates
    for j = 1:numAngleCandidates
        tempLength = lenCandidates(i);
        tempAngle = angleCandidates(j);
        D_alpha = 1 + tempAngle*1.34e-4 + tempAngle^2*(-6.75e-2) + tempAngle^3*1.64e-3;
        
        tempDeltaY = tempLength * D_alpha * sin(tempAngle / 2);

        if( abs(tempDeltaY - deltaY) < deltaYTolerance)
            validLenIndices = [validLenIndices i];
            validAngleIndices = [validAngleIndices j];
        end
    end

end

numCurves = length(validAngleIndices);
all_curves.X = zeros(numCurves,1200);
all_curves.Y = zeros(numCurves,1200);
for k=1:numCurves
    % createElementary(init_pos, init_tan, k1,curv_len)
    curv_len = lenCandidates( validLenIndices(k) );
    angle = angleCandidates( validAngleIndices(k) );
    tempK1 = 2 * angle / curv_len;

    [C1, C2] = createElementary(init_pos,init_tan,tempK1,curv_len);

    all_curves.X(k,:) = [C1.allX C2.allX];
    all_curves.Y(k,:) = [C1.allY C2.allY];
end

% plot curves
figure;
for j = 1:numCurves
    plot(all_curves.X(j,:),all_curves.Y(j,:))
    hold on
end
plot(init_pos(1),init_pos(2),'*')
hold on
plot(final_pos(1),final_pos(2),'*','MarkerSize',10)
hold on
quiver(final_pos(1),final_pos(2), 5*cos(final_tan), 5*sin(final_tan),'LineWidth',2)
grid on

% plot the road
% arcSegment(init_pos,init_tan, radius, length,curv_sign)
arcSeg = arcSegment(init_pos,init_tan,radius,path_length,1);
[x , y] = arcSeg.getXY();
hold on
plot(x,y,'LineWidth',3)
% ylim(xlim)
xlabel('X(m)')
ylabel('Y(m)')
title('Valid Paths')