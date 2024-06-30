function plotBezierAnalysis(bezierTrajectory, xyPairs, allTangents, allCurvatures, shiftedCoords1, shiftedCoords2,...
    arcSplinePos,arcSplineErrors,arcSplineHeadingErrors,arcSplineCurvatureErrors)
    % Compute Bézier errors
    ground_truth_xy = xyPairs;
    measurement_xy = [bezierTrajectory.allX bezierTrajectory.allY];
    [~, ~, bezierErrors] = computeSegmentError(measurement_xy, ground_truth_xy);
    
    % Plot Bézier Trajectory and Road Centerline with Lane Boundaries
    figure;
    h1 = plot(bezierTrajectory.allX, bezierTrajectory.allY, "LineWidth", 1.5, "DisplayName", "Bézier Trajectory");
    hold on;
    h2 = plot(ground_truth_xy(:,1), ground_truth_xy(:,2), "LineWidth", 1.5, "DisplayName", "Road Centerline");
    h3 = plot(shiftedCoords1(:,1), shiftedCoords1(:,2), '--', 'Color', [0 0 0], 'DisplayName', 'Lane Boundary', 'LineWidth', 1);
    h4 = plot(arcSplinePos(:,1),arcSplinePos(:,2),"LineWidth",1.5,"DisplayName","Arc Spline Trajectory");
    plot(shiftedCoords2(:,1), shiftedCoords2(:,2), '--', 'Color', [0 0 0], 'LineWidth', 1);
    xlabel("xEast (m)", "FontSize", 13);
    ylabel("yNorth (m)", "FontSize", 13);
    grid on;
    title("Bézier Trajectory and Road Centerline", "FontSize", 13);
    axis equal;
    legend([h1 h2 h3 h4]);
    
    xAxisArcSpline = 10*(0:0.01:(length(arcSplineErrors)-1) / 100);
    % Plot Bézier Errors
    xAxisBezier = 10*(linspace(0,length(arcSplineErrors)/100,length(bezierErrors)));
    
    figure;
    plot(xAxisBezier, bezierErrors, "LineWidth", 1.5, "DisplayName", "Bézier Error");
    hold on
    plot(xAxisArcSpline,arcSplineErrors, "LineWidth", 1.5, "DisplayName", "Arc Spline Error");
    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Distance to centerline (m)", "FontSize", 13);
    grid on;
    title("Euclidean Distance Error of Trajectories", "FontSize", 13);
    legend();
    
    [~, index] = findClosestPointOnLine(bezierTrajectory.allX(end)...
        , bezierTrajectory.allY(end), bezierTrajectory.allTangent(end) + pi/2, xyPairs);
    
    % Downsample Tangent and Curvature Arrays
    tangentsGT = allTangents(1:index);
    curvaturesGT = allCurvatures(1:index);
    n1 = length(bezierTrajectory.allTangent);
    n2 = length(tangentsGT);
    
    newIndices = linspace(1, n2, n1);
    downsampledArrayTangent = interp1(1:n2, tangentsGT, newIndices)';
    downsampledArrayCurvature = interp1(1:n2, curvaturesGT, newIndices)';
    
    % Plot Heading Errors
    bezierHeadingErrors = rad2deg(downsampledArrayTangent - bezierTrajectory.allTangent);
    figure;
    plot(xAxisBezier, bezierHeadingErrors, "LineWidth", 1.5, "DisplayName", "Bézier Error");
    hold on
    plot(xAxisArcSpline,arcSplineHeadingErrors, "LineWidth", 1.5, "DisplayName", "Arc Spline Error");
    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Heading Error (°)", "FontSize", 13);
    grid on;
    title("Heading Error of Trajectories", "FontSize", 13);
    legend();
    
    % Plot Curvature Errors
    bezierCurvatureErrors = downsampledArrayCurvature - bezierTrajectory.allCurvature;
    figure;
    plot(xAxisBezier, bezierCurvatureErrors, "LineWidth", 1.5, "DisplayName", "Bézier Error");
    hold on
    plot(xAxisArcSpline,arcSplineCurvatureErrors, "LineWidth", 1.5, "DisplayName", "Arc Spline Error");
    xlabel("Trajectory Length (m)", "FontSize", 13);
    ylabel("Curvature Error (m^{-1})", "FontSize", 13);
    grid on;
    title("Curvature Error of Bézier Trajectory", "FontSize", 13);
    legend();
end
