function [angles] = findArcAngles(tempClothoid)


singleArcNumPoints = floor(tempClothoid.segmentLength / 0.01 / (tempClothoid.numArcs));
inbetweenArcsPoints = singleArcNumPoints-1;

numAngles = tempClothoid.numArcs + 1;
angles = zeros(numAngles,1);

for i = 1:numAngles

    if (i == 1) || (i == numAngles)
        singleArcNumPoints = floor(inbetweenArcsPoints / 2); 
    else
        singleArcNumPoints = inbetweenArcsPoints;
    end
    center = tempClothoid.arcCenters(i,:);
    startX = tempClothoid.allX( (i-1)*singleArcNumPoints + 1 );
    startY = tempClothoid.allY( (i-1)*singleArcNumPoints + 1 );

    endX = tempClothoid.allX( i*singleArcNumPoints );
    endY = tempClothoid.allY( i*singleArcNumPoints );

    startAngle = mod(atan2(center(2)-startY, center(1)-startX ), 2*pi);
    endAngle = mod(atan2(center(2)-endY, center(1)-endX ),2*pi);
    diffAngles = endAngle - startAngle;
    if (diffAngles > pi)
        angles(i) = 2*pi - diffAngles;
    elseif( diffAngles < -pi )
        angles(i) = 2*pi + diffAngles;
    else
        angles(i) = abs(diffAngles);
    end
end

end

