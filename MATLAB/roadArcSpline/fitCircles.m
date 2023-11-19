function [arcSegments] = fitCircles(curvature,centers,thetas)

allPoints = [];
for k = 1:length(curvature)
    
    start_angle = thetas(k+1);
    end_angle = thetas(k+2);

    start_angle(start_angle < 0) = start_angle(start_angle < 0) + 2*pi;
    end_angle(end_angle < 0) = end_angle(end_angle < 0) + 2*pi;
    
    radius = abs(1/curvature(k));
    temp = arcSegment_v2(centers(k,:), rad2deg(start_angle), rad2deg(end_angle), radius);

    temp.plotArc();

    arcSegments{k} = temp;

    allPoints = [allPoints; temp.Coordinates];
end

end

