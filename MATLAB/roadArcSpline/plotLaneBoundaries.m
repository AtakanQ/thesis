function h1 = plotLaneBoundaries(all_clothoids_LB, indices)

for i = 1:numel(indices)
    j = indices(i);
    for k = 1:numel(all_clothoids_LB{j})
        plot(all_clothoids_LB{j}(k).allX,all_clothoids_LB{j}(k).allY,'--','Color',[0 0 0],'LineWidth',1)
        hold on
    end
end
h1 = plot(NaN,NaN,'Color',[0 0 0],'DisplayName','Lane Boundary');

end

