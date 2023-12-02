function inspectSegment(segment_idx, curvatures, L, ...
    arcSegments, all_clothoids,errors,xEast,yNorth,theta)
figure;
plot(errors{segment_idx})
% title(strcat( 'Error for radius: ',num2str(abs( 1/curvature_MVRC(segment_idx)) ),...
%     'Segment Length:  ', num2str(L(segment_idx)))  )
title(['Error for radius: ',num2str(abs( 1/curvatures(segment_idx)) ),...
    '  ','Segment Length:  ', num2str(L(segment_idx))])  

figure;
arcSegments{segment_idx}.plotArc();
hold on
all_clothoids(segment_idx).plotPlain();
title(['Generated Arc and Clothoid for Specified Segment ', num2str(segment_idx)])
axis equal
% heading_change_for_specified_index = rad2deg(all_clothoids(segment_idx).final_tan -...
%     all_clothoids(segment_idx).init_tan)

heading_change_for_specified_index_for_LINE = rad2deg(theta(segment_idx) - ...
    atan2(yNorth(segment_idx + 1 )-yNorth(segment_idx),...
    (xEast(segment_idx+1)-xEast(segment_idx)) ) )
end