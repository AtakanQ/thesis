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
xlabel('Arc Length, s');
ylabel('Curvature, k');
title('Curvature of a Clothoid Curve and Arc Approximated Clothoid Curve');

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
text(startingLength, 0, "  L/(2n)", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 12);
for i = 2:order
    if i == 2
        text(startingLength+(i-1)*L/order, 0, "  L/(2n) + L/n", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 10);
    else
        text(startingLength+(i-1)*L/order, 0, "  L/(2n) + " + num2str(i-1) + "L/n", 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 10);
    end
    
end
text(L, 0, '  L', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center', 'FontSize', 12);

% Adjusting plot limits if necessary
ylim([-0.1*max(k), 1.1*max(k)]);
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
legend('Clothoid','Arc Spline Approximation');