function plotBezierAnalysisArray(bezierTrajectories, xyPairs, allTangents, allCurvatures, ...
    shiftedCoords1, shiftedCoords2,arcSplinePos,arcSplineErrors,arcSplineHeadingErrors,arcSplineCurvatureErrors)
    % Number of Bézier trajectories
    numTrajectories = length(bezierTrajectories);
    
    % Plot Bézier Trajectories and Road Centerline with Lane Boundaries
    figure;
    hold on;
    h = gobjects(1, numTrajectories); % Preallocate graphics object array for legend handles
    for i = 1:numTrajectories
        h(i) = plot(bezierTrajectories(i).allX, bezierTrajectories(i).allY, "LineWidth", 1.5, "DisplayName", sprintf("Bézier Trajectory %d", i));
    end
    h3 = plot(xyPairs(:,1), xyPairs(:,2), "LineWidth", 1.5, "DisplayName", "Road Centerline");
    h4 = plot(arcSplinePos(:,1),arcSplinePos(:,2),"LineWidth",1.5,"DisplayName","Arc Spline Trajectory");
    plot(shiftedCoords1(:,1), shiftedCoords1(:,2), '--', 'Color', [0 0 0], 'DisplayName', 'Lane Boundary', 'LineWidth', 1);
    plot(shiftedCoords2(:,1), shiftedCoords2(:,2), '--', 'Color', [0 0 0], 'LineWidth', 1);
    xlabel("xEast (m)", "FontSize", 13);
    ylabel("yNorth (m)", "FontSize", 13);
    grid on;
    title("Trajectories and Road Centerline", "FontSize", 13);
    axis equal;
    minY = min(arcSplinePos([1 length(arcSplinePos)],2));
    maxY = max(arcSplinePos([1 length(arcSplinePos)],2));
    ylim([minY-5 maxY+5])
    legend([h h3 h4]);
    
    index = zeros(numel(bezierTrajectories),1);
    for i = 1:numel(bezierTrajectories)
        [~, index(i)] = findClosestPointOnLine(bezierTrajectories(i).allX(end)...
        , bezierTrajectories(i).allY(end), bezierTrajectories(i).allTangent(end) + pi/2, xyPairs);
    end

    % Plot Bézier Errors for all trajectories
    ground_truth_xy = xyPairs;
    figure;
    hold on;
    for i = 1:numTrajectories
        measurement_xy = [bezierTrajectories(i).allX, bezierTrajectories(i).allY];
        [~, ~, bezierErrors] = computeSegmentError(measurement_xy, ground_truth_xy);

        n2 = length(bezierErrors);
        n1 = index(i);

        newIndices = linspace(1, n2, n1);
        bezierErrors = interp1(1:n2, bezierErrors, newIndices)';
        
        xAxisBezier = 0:0.01:(index(i) - 1) / 100;

        plot(xAxisBezier, bezierErrors, "LineWidth", 1.5, "DisplayName", sprintf("Bézier Error %d", i));
    end
    n2 = length(arcSplineErrors);
    n1 = index(ceil(length(index) / 2) );
    newIndices = linspace(1, n2, n1);
    arcSplineErrors = interp1(1:n2, arcSplineErrors, newIndices)';
        
    plot(0:0.01:(n1-1)/100,arcSplineErrors, "LineWidth", 1.5, "DisplayName", sprintf("Arc Spline Error"));

    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Distance to centerline (m)", "FontSize", 13);
    grid on;
    title("Euclidean Distance Error of Bézier Trajectories", "FontSize", 13);
    legend('Location','best');
    



    % Downsample Tangent and Curvature Arrays
    figure;
    hold on;
    for i = 1:numTrajectories

        tangentsGT = allTangents(1:(index(i)));

        n2 = length(bezierTrajectories(i).allTangent);
        n1 = index(i);

        newIndices = linspace(1, n2, n1);
        bezierTrajectories(i).allTangent = interp1(1:n2, bezierTrajectories(i).allTangent, newIndices)';
        
        % Plot Heading Errors for all trajectories
        bezierHeadingErrors = rad2deg(tangentsGT - bezierTrajectories(i).allTangent');
        xAxisBezier = 0:0.01:(length(bezierHeadingErrors) - 1) / 100;
        plot(xAxisBezier, bezierHeadingErrors, "LineWidth", 1.5, "DisplayName", sprintf("Bézier Error %d", i));
    end
    n2 = length(arcSplineHeadingErrors);
    n1 = index(ceil(length(index) / 2) );
    newIndices = linspace(1, n2, n1);
    arcSplineHeadingErrors = interp1(1:n2, arcSplineHeadingErrors, newIndices)';
    plot(0:0.01:(n1-1)/100,arcSplineHeadingErrors, "LineWidth", 1.5, "DisplayName", sprintf("Arc Spline Error"));
    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Heading Error (°)", "FontSize", 13);
    grid on;
    title("Heading Error of Bézier Trajectories", "FontSize", 13);
    legend('Location','best');
    
    figure;
    hold on;
    for i = 1:numTrajectories
        curvaturesGT = allCurvatures(1:(index(i)));

        n2 = length(bezierTrajectories(i).allCurvature);
        n1 = index(i);
        newIndices = linspace(1, n2, n1);
        bezierTrajectories(i).allCurvature = interp1(1:n2, bezierTrajectories(i).allCurvature, newIndices)';
        
        % Plot Curvature Errors for all trajectories
        bezierCurvatureErrors = curvaturesGT - bezierTrajectories(i).allCurvature';
        xAxisBezier = 0:0.01:(length(bezierCurvatureErrors) - 1) / 100;
        plot(xAxisBezier, bezierCurvatureErrors, "LineWidth", 1.5, "DisplayName", sprintf("Bézier Error %d", i));
    end

    n2 = length(arcSplineCurvatureErrors);
    n1 = index(ceil(length(index) / 2) );
    newIndices = linspace(1, n2, n1);
    arcSplineCurvatureErrors = interp1(1:n2, arcSplineCurvatureErrors, newIndices)';
    plot(0:0.01:(n1-1)/100,arcSplineCurvatureErrors, "LineWidth", 1.5, "DisplayName", sprintf("Arc Spline Error"));
    
    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Curvature Error (m^{-1})", "FontSize", 13);
    grid on;
    title("Curvature Error of Bézier Trajectories", "FontSize", 13);
    legend('Location','best');
end
