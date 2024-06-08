clear

addpath("..\roadArcSpline\")
load('C:\Users\Atakan\Documents\GitHub\thesis\MATLAB\roadArcSpline\last_work_V8.mat')
set(groot, 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesZGrid', 'on');
%% Compute rms and max error of a picked shifted lane (arc spline)

RMS_errors = zeros(numel(segments{1}),1);
MAX_errors = zeros(numel(segments{1}),1);
average_curvatures = zeros(numel(RMS_errors),1);
curveLengths = zeros(numel(RMS_errors),1);
for i = 1:numel(segments{1})
    if(segments{1}(i).type == "clothoid")
        currX = segments{1}(i).allX';
        currY = segments{1}(i).allY';
    else
        currX = segments{1}(i).allX;
        currY = segments{1}(i).allY;
    end
    currX_GT = [all_clothoids{1}(i).allX'; all_clothoids{1}(i+1).allX'; all_clothoids{1}(i+2).allX'];
    currY_GT = [all_clothoids{1}(i).allY'; all_clothoids{1}(i+1).allY'; all_clothoids{1}(i+2).allY'];
    
    average_curvatures(i) = (all_clothoids{1}(i+1).init_curv + all_clothoids{1}(i+1).final_curv)/2;
    curveLengths(i) = all_clothoids{1}(i+1).curv_length;
    [RMS_errors(i), MAX_errors(i), ~] = ...
        computeSegmentError([currX currY],[currX_GT currY_GT]);

end

%% Plot
figure;
plot(RMS_errors,'DisplayName', 'RMS Error','LineWidth',1.5)
title('RMS Error Between Approximated Base Lane and Ground Truth','FontSize',13)
ylabel('Error (m)','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(RMS_errors)])

figure;
plot(MAX_errors,'DisplayName', 'Max Error','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
title('Maximum Error Between Approximated Base Lane and Ground Truth','FontSize',13)
ylabel('Error (m)','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(RMS_errors)])

%% Plot with average curvatures
figure;
plot(RMS_errors,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
ylabel('Error (m)','FontSize',13)
yyaxis right
ylabel('Curvature (m^-^1)','FontSize',13)
ylim([-12e-4 12e-4])
plot(average_curvatures,'DisplayName', 'Curvature','LineWidth',1.5,'Color',[0.4940, 0.1840, 0.5560])
title('RMS Error and Average Curvature of Segments','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(RMS_errors)])

figure;
plot(MAX_errors,'DisplayName', 'Max Error','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
hold on
ylabel('Error (m)','FontSize',13)
yyaxis right
ylabel('Curvature (m^-^1)','FontSize',13)
ylim([-12e-4 12e-4])
plot(average_curvatures,'DisplayName', 'Curvature','LineWidth',1.5,'Color',[0.4940, 0.1840, 0.5560])
title('Max Error and Average Curvature of Segments','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(MAX_errors)])

%% Plot curve length and error
figure;
plot(RMS_errors,'DisplayName', 'RMS Error','LineWidth',1.5)
hold on
ylabel('Error (m)','FontSize',13)
yyaxis right
ylabel('Curve Length (m)','FontSize',13)
% ylim([-12e-4 12e-4])
plot(curveLengths,'DisplayName', 'Segment Length','LineWidth',1.5,'Color',[0.6350, 0.0780, 0.1840])
title('RMS Error and Curve Lengths of Segments','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(RMS_errors)])

figure;
plot(MAX_errors,'DisplayName', 'Max Error','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
hold on
ylabel('Error (m)','FontSize',13)
yyaxis right
ylabel('Curve Length (m)','FontSize',13)
% ylim([-12e-4 12e-4])
plot(curveLengths,'DisplayName', 'Segment Length','LineWidth',1.5,'Color',[0.6350, 0.0780, 0.1840])
title('Max Error and Curve Lengths of Segments','FontSize',13)
xlabel('Segment Index','FontSize',13)
legend()
xlim([1 numel(MAX_errors)])
