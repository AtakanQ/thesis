close all
clear

%% Path expression
% enter challenging values here
% v = 100 m/s radius = 490meters min_path_length = 55m 
% Emax = 4% fmax = 0.12

radius = 490; %m 
% radius = 1000; %m 
path_length = 98; %m given
theta = path_length / radius;

% divide the road to 10. This may change according
% to length
numPlanningSegments = 3;
theta = theta/numPlanningSegments;

% initial and final points
init_pos = [0 0];
init_tan = 0;
final_pos = [radius*sin(theta) radius*(1-cos(theta))]; %first_final_pos
final_tan = theta; %first_final_tan

real_init_pos = init_pos;
real_init_tan = init_tan;

%tolerances
deltaYTolerance = 0.2;

% number of total candidates are calculated here
numLenCandidates = 100;

all_curves = cell(1,numPlanningSegments);

figure;
for m = 1:numPlanningSegments
    minLength = norm(init_pos - final_pos);
    small_angle = atan((final_pos(2)-init_pos(2))/(final_pos(1)-init_pos(1)) ) - init_tan;
    lower_side = minLength * cos(small_angle);
    right_side = minLength * sin(small_angle);
    % maxLength = abs(final_pos(1) - init_pos(1)) + abs(final_pos(2) - init_pos(2));
    maxLength = lower_side + right_side;
    lenCandidates = linspace(minLength,maxLength,numLenCandidates);

    validLenIndices = [];
    
    % deltaY = abs(final_pos(2) - init_pos(2));
    deltaY = minLength * sin(small_angle);
    for i = 1:numLenCandidates
            tempLength = lenCandidates(i);
            tempAngle = final_tan - init_tan;
            D_alpha = 1 + tempAngle*1.34e-4 + tempAngle^2*(-6.75e-2) + tempAngle^3*1.64e-3;
            
            tempDeltaY = tempLength * D_alpha * sin(tempAngle / 2);
    
            if( abs(tempDeltaY - deltaY) < deltaYTolerance)
                validLenIndices = [validLenIndices i];
            end
    
    end

    numCurves = length(validLenIndices);
    all_curves{m}.X = zeros(numCurves,1200);
    all_curves{m}.Y = zeros(numCurves,1200);

    for k=1:numCurves
        % createElementary(init_pos, init_tan, k1,curv_len)
        curv_len = lenCandidates( validLenIndices(k) );
        k1 = 2 * (final_tan - init_tan) / curv_len;
    
        [C1, C2] = createElementary(init_pos,init_tan,k1,curv_len);
    
        all_curves{m}.X(k,:) = [C1.allX C2.allX];
        all_curves{m}.Y(k,:) = [C1.allY C2.allY];
    end
    
    init_pos = [all_curves{m}.X(ceil(end/2),end) all_curves{m}.Y(ceil(end/2),end)];
    init_tan = final_tan;
    final_pos = [radius*sin( (m+1) * theta) radius*(1-cos( (m+1) * theta))];
    final_tan = final_tan + theta;
    
    for n = 1:numCurves
        plot(all_curves{m}.X(n,:),all_curves{m}.Y(n,:))
        hold on
    end
    
end
% numCurves = length(validLenIndices);
% all_curves.X = zeros(numCurves,1200);
% all_curves.Y = zeros(numCurves,1200);
% for k=1:numCurves
%     % createElementary(init_pos, init_tan, k1,curv_len)
%     curv_len = lenCandidates( validLenIndices(k) );
%     angle = angleCandidates( validAngleIndices(k) );
%     tempK1 = 2 * angle / curv_len;
% 
%     [C1, C2] = createElementary(init_pos,init_tan,tempK1,curv_len);
% 
%     all_curves.X(k,:) = [C1.allX C2.allX];
%     all_curves.Y(k,:) = [C1.allY C2.allY];
% end

% plot curves
% figure;
% for j = 1:numCurves
%     plot(all_curves.X(j,:),all_curves.Y(j,:))
%     hold on
% end
% plot(init_pos(1),init_pos(2),'*')
% hold on
% plot(final_pos(1),final_pos(2),'*','MarkerSize',10)
% hold on
% quiver(final_pos(1),final_pos(2), 5*cos(final_tan), 5*sin(final_tan),'LineWidth',2)
% grid on

% plot the road
% arcSegment(init_pos,init_tan, radius, length,curv_sign)

arcSeg = arcSegment(real_init_pos,real_init_tan,radius,path_length,1);
[x , y] = arcSeg.getXY();
hold on

plot(x,y,'LineWidth',1,'Color',[1 0 0])
xlabel('X(m)')
ylabel('Y(m)')
title('Valid Paths')
grid on
% ylim(xlim)

% print radii
% for k = 1:6
%     disp([num2str(k), '. arc segment radius: ', num2str(1/C1.arcSegments(k).radius)])
% end
