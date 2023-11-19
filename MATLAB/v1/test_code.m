close all
clear
init_pos = [0 0];
init_tan = 0; %45 deg
radius = 20;
length = 15;
curv_sign = 1;
% tic
% seg = arcSegment(init_pos,init_tan, radius, length,curv_sign);
% seg.getXY();
% toc
% seg.plotWholeCircle();
% seg.plotSegment();

% dummyArcSeg = arcSegment;
% cloth = clothoid(init_pos,init_tan,0.5,0,5,5,dummyArcSeg);
% cloth.generateArcSegments()
% cloth.plotClothoidWithCircles()
% cloth.plotPlain()

[C1, C2] = createElementary([0 0],0,0.015,100);
lastPos = [C2.allX(end) C2.allY(end)];
lastTan = C2.final_tangent;
radius = 999;
length = 10;
curv_sign = 0;
lineSeg = arcSegment(lastPos,lastTan,radius,length,curv_sign);
lineSeg.getXY() 
lineLastPos = [lineSeg.x_coor(end) lineSeg.y_coor(end)];

[C3, C4] = createElementary(lineLastPos,lastTan, -0.015,100);

allX = [C1.allX C2.allX lineSeg.x_coor C3.allX C4.allX];
allY = [C1.allY C2.allY lineSeg.y_coor C3.allY C4.allY]; 

figure;
plot(allX,allY);
title("Bi-elementary path")
xlabel("x(m)")
ylabel("y(m)")
