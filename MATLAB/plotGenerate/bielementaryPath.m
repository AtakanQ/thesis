
addpath("..\roadArcSpline\")

S1 = 5;
S2 = 5;
S3 = 10;
S4 = 5;
S5 = 5;
k1 = 0.1;
dataPointSparsity = 0.1;
init_pos = [0 0];
init_tan = 0;
final_curvature = k1;

 % clothoid_v2( ...
 %               init_pos, ...
 %               init_tan, ...
 %               init_curvature, ...
 %               final_curvature,...
 %               curv_length, ...
 %               dataPointSparsity ...
 %               )

C1 = clothoid_v2(init_pos,init_tan,0,k1,S1,dataPointSparsity);
C2 = clothoid_v2([C1.allX(end) C1.allY(end)],C1.final_tan,k1,0,S2,dataPointSparsity);
lineEnd = S3*[cos(C2.final_tan) sin(C2.init_tan)] + [C2.allX(end) C2.allY(end)];
C3 = clothoid_v2(lineEnd,C2.final_tan,0,-k1,S4,dataPointSparsity);
C4 = clothoid_v2([C3.allX(end) C3.allY(end)],C3.final_tan,-k1,0,S5,dataPointSparsity);

deltaY = [C1.allY(1:(end-1)) C2.allY(1:(end-1))];
deltaY = [deltaY linspace(deltaY(end),lineEnd(2)-0.05,S3*(1/dataPointSparsity))];
deltaY = [deltaY C3.allY(1:(end-1)) C4.allY(1:(end-1))];
xAxis = linspace(0,length(deltaY)*dataPointSparsity,length(deltaY));

subplot(2,1,1)
plot(xAxis,deltaY,'LineWidth',1.5,'Color','b')
title("$\Delta$Y over Bi-elementary Trajectory",'Interpreter','latex',"FontSize",13)
xlabel("Arc Length (m)","FontSize",13)
ylabel("$\Delta$Y (m)",'Interpreter','latex',"FontSize",13)
hold on
xline(S1,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3+S4,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3+S4+S5,'--','Color',[0 0 0],'LineWidth',1)
text(S1/2,deltaY(end)/2,'C1','FontSize',13,'HorizontalAlignment','center')
text(S1+S2/2,deltaY(end)/2,'C2','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3/2,deltaY(end)*2/3,'L','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3+S4/2,deltaY(end)/2,'C3','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3+S4+S5/2,deltaY(end)/2,'C3','FontSize',13,'HorizontalAlignment','center')
% legend('','Interpreter','latex');
xAxisLengths = [0 S1 S1+S2 S1+S2+S3 S1+S2+S3+S4 S1+S2+S3+S4+S5];
curvatures = [0 k1 0 0 -k1 0];
subplot(2,1,2)
plot(xAxisLengths,curvatures,'LineWidth',1.5,'Color','r')
title("Curvature over Bi-elementary Trajectory",'Interpreter','latex',"FontSize",13)
xlabel("Arc Length (m)","FontSize",13)
ylabel("$\kappa(s)$ (m)",'Interpreter','latex',"FontSize",13)
xline(S1,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3+S4,'--','Color',[0 0 0],'LineWidth',1)
xline(S1+S2+S3+S4+S5,'--','Color',[0 0 0],'LineWidth',1)
text(S1/2,0,'C1','FontSize',13,'HorizontalAlignment','center')
text(S1+S2/2,0,'C2','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3/2,k1(end)/2,'L','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3+S4/2,0,'C3','FontSize',13,'HorizontalAlignment','center')
text(S1+S2+S3+S4+S5/2,0,'C3','FontSize',13,'HorizontalAlignment','center')
