addpath('../roadArcSpline/')

init_pos = [0 0];
init_tan = 0;
init_curvature = 0;
final_curvature = .01;
curv_length = 100;

myClothoidZeroInit = clothoid_v2( ...
               init_pos, ...
               init_tan, ...
               init_curvature, ...
               final_curvature,...
               curv_length ...
               );
figure;
myClothoidZeroInit.plotPlain()
axis equal
xlabel("xEast (m)",'FontSize',13)
ylabel("yNorth (m)",'FontSize',13)
title_text = ['Zero Initial Valued Clothoid'];
grid on
title(title_text,'FontSize',13)

%%  decreasing curvature

init_pos = [0 0];
init_tan = 0;
init_curvature = 0.01;
final_curvature = 0;
curv_length = 100;

myClothoidPosInit = clothoid_v2( ...
               init_pos, ...
               init_tan, ...
               init_curvature, ...
               final_curvature,...
               curv_length ...
               );

figure;
myClothoidPosInit.plotPlain()
axis equal
xlabel("xEast (m)",'FontSize',13)
ylabel("yNorth (m)",'FontSize',13)
title_text = ['Positive Initial Valued Clothoid'];
grid on
title(title_text,'FontSize',13)

%% plot on top of each other

figure;
color = [1 0 0];
legend_plt = 'Positive Initial';
myClothoidPosInit.plotPlain(color,legend_plt)
hold on
color = [0 0 1];
legend_plt = 'Zero Initial';
myClothoidZeroInit.plotPlain(color,legend_plt)
axis equal
xlabel("xEast (m)",'FontSize',13)
ylabel("yNorth (m)",'FontSize',13)
title_text = ['Two Clothoids on Same Plot'];
legend('FontSize',13)
grid on
title(title_text,'FontSize',13)

