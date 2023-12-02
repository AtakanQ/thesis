function curvature_differences = inspectArcs(curvatures, L,rms_errors,MVRC)
curvature_differences = abs(diff(curvatures))*1000;

if MVRC
    figure;
    plot(curvature_differences,'Color',[0 0 1])
    hold on
    ylabel('Curvature difference*1000 and RMS error')
    plot(rms_errors(2:end),'Color',[1 0 0])
    ylabel('RMS error and Curvature*1000')
    yyaxis right
    plot(L(2:end-1),'Color',[0 0 0])
    legend('Abs Curvature differences', 'RMS errors','Segment length');
    title('Absolute Curvature Differences for MVRC Method and RMS Errors')
    grid on
    xlabel('Segment index')
else
    figure;
    plot(curvature_differences,'Color',[0 0 1])
    hold on
    ylabel('Curvature difference*1000 and RMS error')
    plot(rms_errors,'Color',[1 0 0])
    ylabel('RMS error and Curvature*1000')
    yyaxis right
    plot(L,'Color',[0 0 0])
    legend('Abs Curvature differences', 'RMS errors','Segment length');
    title('Absolute Curvature Differences for GT and RMS Errors')
    grid on
    xlabel('Segment index')
end

end