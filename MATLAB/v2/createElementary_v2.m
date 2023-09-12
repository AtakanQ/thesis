function [C1, C2] = createElementary_v2(init_pos, init_tan, k1,curv_len,init_curvature,final_curvature)
    
    
    dummyArcSeg = arcSegment;

    C1 = clothoid(init_pos,init_tan,init_curvature,k1,curv_len/2,5,dummyArcSeg);
    C1.generateArcSegments();
    final_point_C1 = [C1.allX(end) C1.allY(end)];
    final_tangent_C1 = C1.final_tangent;

    C2 = clothoid(final_point_C1,final_tangent_C1,k1,final_curvature,curv_len/2,5,dummyArcSeg);
    C2.generateArcSegments();
    
    A = [C1.allX' C1.allY'; C2.allX' C2.allY'];
    distances = sqrt( sum(  diff( A ).^2,2 ));
    arc_len = zeros(length(A)-1,1);
    for j = 1:(length(A) -1 )
        arc_len(j) = sum(distances(1:j)); 
    end
    
    
    % figure;
    % plot(arc_len,A(1:(end - 1),2),'Color',[0 0 1]);
    % title('Y position evolution')
    % xlabel('arc length [m]')
    % ylabel('Y-position [m]')
    % grid on
    % 
    % figure;
    % plot(C1.allX,C1.allY,'Color',[0 0 1]);
    % hold on
    % plot(C2.allX,C2.allY,'--','Color',[0 0 0]);
    % grid on
    % title('Elementary Path')
    % xlabel('x')
    % ylabel('y')
end