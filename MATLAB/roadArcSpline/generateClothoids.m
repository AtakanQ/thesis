function [all_clothoids] = generateClothoids(xEast,yNorth,theta,k,dk,L)
%generate clothoid between consecutive points. 
num_clothoids = length(k);

% figure;
for i=1:num_clothoids
    init_pos = [xEast(i) yNorth(i)]; 
    init_tan = theta(i); 

    %OBSOLETE WITH clothoid_v2
    % all_clothoids(i) = clothoid(init_pos,init_tan,k(i),k(i)+dk(i)*L(i),...
    %     L(i),20,dummy_arcSeg);
    if i == length(k)
        all_clothoids(i) = clothoid_v2(init_pos, init_tan, ...
            k(i), k(i)+dk(i)*L(i),L(i),25000 );
    else
        all_clothoids(i) = clothoid_v2(init_pos, init_tan, ...
            k(i), k(i+1),L(i),25000 );        
    end

    %OBSOLETE WITH clothoid_v2
    % all_clothoids(i).generateArcSegments();

    % all_clothoids(i).plotPlain();
    % hold on
    % axis equal
    % v2_cloth = clothoid_v2(init_pos, init_tan,k(i), k(i)+dk(i)*L(i),L(i),1000 );
    % figure;
    % v2_cloth.plotPlain();
  end
% title('Clothoid Path')
% xlabel('x (m)')
% ylabel('y (m)')
% axis equal
% grid on
end

