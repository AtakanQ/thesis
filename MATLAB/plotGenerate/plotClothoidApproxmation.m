% Parameters for the clothoid
a = 0.001;    % This constant determines the rate at which curvature changes
L = 50;     % Total arc length

order = 5;

% Generate arc length values from -L/2 to L/2
s = linspace(0, L, 1000);

k1 = 0.01;
% Calculate the curvature k(s) = a * s
k = k1 + a * s;

% Create the plot
figure;
plot(s, k, 'LineWidth', 2,'DisplayName','Clothoid');
grid on;
xlabel('Arc Length, s','FontSize',12);
ylabel('Curvature, k','FontSize',12);
title('Curvature of a Clothoid Curve and Arc Approximated Clothoid Curve','FontSize',12);

% Annotating the plot with k1, k2, and L
hold on;

% Points to annotate k_1 and k_2
k1 = k(1);
k2 = k(end);

% Plotting points
% plot(0, k1, 'ro'); % Point at start
% plot(L, k2, 'ro');  % Point at end

startingLength = L / (2*order);
plot([0 startingLength], [k1 k1], 'Color',[1 0.5 0],'LineWidth',2,'DisplayName','Arc Spline Approximation'); % Point at start 

for i = 1:(order-1)
    plot([startingLength+(i-1)*L/order startingLength+(i)*L/order], [k1+(i-0.5)*(k2-k1)/order k1+(i-0.5)*(k2-k1)/order], 'Color',[1 0.5 0],'LineWidth',2);
end

plot([startingLength +  (order-1)*L/order L], [k2 k2], 'Color',[1 0.5 0],'LineWidth',2);  % Point at end



% Text annotations
% text(0, k1, '  k_1', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', 'FontSize', 12);
% text(0, k2, '  k_2', 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right', 'FontSize', 12);
text(startingLength, 0, "$\frac{L}{2n}$", 'Interpreter', 'latex', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 11);
for i = 2:order
    % if i == 2
    %     text(startingLength+(i-1)*L/order, 0, "  L/(2n) + L/n", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 9);
    % else
    %     text(startingLength+(i-1)*L/order, 0, "  L/(2n) + " + num2str(i-1) + "L/n", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 9);
    % end
    if i == 2
        text(startingLength+(i-1)*L/order, 0, '$\frac{L}{2n}$+$\frac{L}{n}$', 'Interpreter', 'latex', 'VerticalAlignment','top', 'HorizontalAlignment', 'center', 'FontSize', 11);
    else
        text(startingLength+(i-1)*L/order, 0, "$\frac{L}{2n}$+$\frac{"+num2str(i-1)+"L}{n}$", 'Interpreter', 'latex' , 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 11);
    end
end
text(L, 0, '$\L$', 'Interpreter', 'latex', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 11);

% Adjusting plot limits if necessary
ylim([-0.1*max(k), 1.3*max(k)]);
xlim([0 L*1.1])
% Remove all ticks and labels
xtck = [0 ];
ytck = [0 k1 ];
ytck_labels = {'0','k_i'};
for i = 1:order
    %create xtick
    xtck(i+1) = startingLength+(i-1)*L/order;

    %create xtick
    ytck(i+2) = k1+(i-0.5)*(k2-k1)/order;
    ytck_labels{i+2} = "k_i + "+ num2str(i) + "(k_f-k_i)/" + num2str(order);
end
xtck = [xtck  L];
ytck = [ytck k2];
ytck_labels = [ytck_labels, {'kf'}];

set(gca, 'XTick', xtck, 'YTick', ytck);
% yticklabels({'0','k_1','k_2'})
yticklabels(ytck_labels)
xticklabels({'0',''})
hold off;   
legend('Clothoid','Arc Spline Approximation','FontSize',12);

%% Plot RMS error between ground truth and approximation
init_pos = [0 0];
init_tan = 0;
init_curvature = 0;
final_curvature = 0.1;
length = 50;
order = 5;
dummyArcSeg = arcSegment;
dataPointSparsity = 0.001; % m
clothoid_GT = clothoid_v2(init_pos,init_tan, init_curvature, final_curvature,...
               length,dataPointSparsity);
ground_truth_xy = [clothoid_GT.allX' clothoid_GT.allY'];

% [rms_error, max_error, errors] = computeSegmentError(measurement_xy,ground_truth_xy)
clothoid_approx_low = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               length,order,dummyArcSeg);

measurement_xy1 = [clothoid_approx_low.allX' clothoid_approx_low.allY'];

[rms_error, max_error, errors1] = computeSegmentError(measurement_xy1,ground_truth_xy);
filtered_data1 = medfilt1(errors1, 10);
figure
plot(filtered_data1)
title("Error for Low Order Approximation",'FontSize',13)
xlabel("Sample Number")

order = 10;
clothoid_approx_high = clothoid(init_pos,init_tan, init_curvature, final_curvature,...
               length,order,dummyArcSeg);

measurement_xy2 = [clothoid_approx_high.allX' clothoid_approx_high.allY'];

[rms_error, max_error, errors2] = computeSegmentError(measurement_xy2,ground_truth_xy);
filtered_data2 = medfilt1(errors2, 10);
figure
plot(filtered_data2)
title("Error for High Order Approximation",'FontSize',13)

filtered_downsampled_data2 = resample(filtered_data2, size(filtered_data1,1), size(filtered_data2,1));

figure;
plot(filtered_data1,'LineWidth',1.5)
hold on
plot(filtered_downsampled_data2,'LineWidth',1.5)
legend("5^t^h Order","10^t^h Order",'FontSize',13)
title("Error for High and Low Order Approximation",'FontSize',13)
xlabel("Sample Number")
ylabel("Error (m)")
ylim([0 0.25])