function [clothoidArray] =  followRoad(nextInitPos,nextInitTan,nextInitCurv,...
    clothoids_GT,plotOn,shiftedCoords1,shiftedCoords2,numSegments)


dataPointSparsity = 0.01;
clothoidArray = [];
for i = 1:numSegments
    tempClothoid = clothoid_v2(nextInitPos, nextInitTan, nextInitCurv,...
        clothoids_GT(i).final_curv,clothoids_GT(i).curv_length,dataPointSparsity);
    clothoidArray = [clothoidArray; tempClothoid];
    nextInitPos = [tempClothoid.allX(end) tempClothoid.allY(end)];
    nextInitTan = tempClothoid.final_tan;
    nextInitCurv = tempClothoid.final_curv;
end

if plotOn
    allX = [];
    allY = [];
    figure;
    for i = 1:numel(clothoidArray)
        p1 = clothoidArray(i).plotPlain([1 0 0],"Arc-spline Trajectory");
        allX = [allX clothoidArray(i).allX];
        allY = [allY clothoidArray(i).allY];
        axis equal
        hold on
    end
    for i = 1:numel(clothoids_GT)
        p2 =clothoids_GT(i).plotPlain([0 0 1],"Road Centerline");
        p2.LineWidth = 0.5;
        axis equal
        hold on
    end
    h4 = plot(shiftedCoords1(:,1),shiftedCoords1(:,2),'--','Color',[0 0 0],'DisplayName','Lane Boundary','LineWidth',1);
    plot(shiftedCoords2(:,1),shiftedCoords2(:,2),'--','Color',[0 0 0],'LineWidth',1)
    
    legend([p1 p2 h4])
    title("Lane Following Mode","FontSize",13)
    xlabel("xEast(m)","FontSize",13)
    ylabel("yNorth (m)","FontSize",13)
    ylim([min(allY) max(allY)])
end

end

