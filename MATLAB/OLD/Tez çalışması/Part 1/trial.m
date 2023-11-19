% tzero = [sqrt(2)/2 sqrt(2)/2];
% tfinal =[cosd(-10) sind(10)];


% rotation matrices
rot_cw = [0 -1; 1 0];
rot_ccw = [0 1; -1 0];

% initial and final location
A = [1 1];
B = [10 3];

%initial and final tangent???
tzero = B-A;
tfinal = A-B;

% tfinal = B-A;

%normalize tangents, magnitudes will be adjusted according to given formula
tzero = tzero / norm(tzero);
tfinal = tfinal / norm(tfinal);

% tzero = [0.9601    0.2796];
% tfinal =[ 0.1953   -0.9807];

% plot the tangents
x_coordinates =[A(1) B(1)];
y_coordinates = [A(2) B(2)];
plot(x_coordinates,y_coordinates,'*');
xlim([0 12]);
ylim([0 6]);
hold on
quiver(x_coordinates(1),y_coordinates(1),tzero(1),tzero(2),'LineWidth',2);
hold on
quiver(x_coordinates(2),y_coordinates(2),tfinal(1),tfinal(2),'LineWidth',2);

% given initial and final curvature values
curve_zero = 0.1;
curve_final = 0;

% calculation parameters
Nt = 10;
Nk = 3;
mtmin = 0.3;
mtmax = 1.7;
mkmin = 0;
mkmax = 10;
dof = norm(A-B); % distance between initial and final point???

tzero_vectors = repmat(tzero,Nt,1);
tfinal_vectors = repmat(tfinal,Nt,1);

%create an array to easily loop
tangent_multiplier = linspace(mtmin,mtmax,Nt);

% create tangent vectors according to given calculation
for i = 1:Nt
    tzero_vectors(i,:) = tzero_vectors(i,:)*tangent_multiplier(i)*dof;
    tfinal_vectors(i,:) = tfinal_vectors(i,:)*tangent_multiplier(i)*dof;
end

%create an array to easily loop
acceleration_multiplier = linspace(mkmin,mkmax,Nk);
acceleration_tangential_multiplier = zeros(1,Nk);

%DOF IS TOO HIGH. MAKING IT SMALLER GIVES A MORE SENSIBLE ROUTE

for k = 1:Nk
    acceleration_tangential_multiplier(k) = acceleration_multiplier(k) * dof;
end

%now create initial and final tangential vectors. First tangential vector
%is bisector of initial heading vector and vector pointing final position. 
%Similarly final tangential vector is bisector of  
tzero = B-A;

tfinal = A-B;
% tfinal = B-A;
angle_zero = atan2d(tzero(2),tzero(1));
new_direction = (45-angle_zero)/2 +angle_zero;

rotation = [cosd(new_direction-angle_zero) sin(new_direction-angle_zero); -sin(new_direction-angle_zero) cos(new_direction-angle_zero)];
tzero = tzero * rotation';

angle_final = atan2d(tfinal(2),tfinal(1));
new_direction = (-10+angle_final)/2;
rotation = [cosd(new_direction-angle_final) sin(new_direction-angle_final); -sin(new_direction-angle_final) cos(new_direction-angle_final)];
tfinal = tfinal* rotation';

% plot the tangents
figure;
x_coordinates =[A(1) B(1)];
y_coordinates = [A(2) B(2)];
plot(x_coordinates,y_coordinates,'*');
xlim([0 12]);
ylim([0 6]);
hold on
quiver(x_coordinates(1),y_coordinates(1),tzero(1),tzero(2),'LineWidth',2);
hold on
quiver(x_coordinates(2),y_coordinates(2),tfinal(1),tfinal(2),'LineWidth',2);

normalized_tzero = tzero * rot_ccw';
normalized_tfinal = tfinal * rot_ccw';

normalized_tzero = normalized_tzero / norm(normalized_tzero);
normalized_tfinal = normalized_tfinal / norm(normalized_tfinal);

acceleration_zero = zeros(Nk,2);
acceleration_final = zeros(Nk,2);
for n = 1:Nk
    acceleration_zero(n,:) =  acceleration_tangential_multiplier(n) * tzero +...
        curve_zero * ( norm(tzero) )^2 * normalized_tzero;

    acceleration_final(n,:) = acceleration_tangential_multiplier(n) * tfinal +...
        curve_final * ( norm(tfinal) )^2 * normalized_tfinal;
end

Curves = cell(4500,1);
P_zero = A;
P_five = B;
curve_position = 1;
for tan_zero = 1:10
    for tan_final = 1:10
        for acc_zero = 1:3
            for acc_final = 1:3
                P_one = P_zero + tzero_vectors(tan_zero,:)/5;

                P_two = acceleration_zero(acc_zero,:)/20 + 2*P_one;

                P_four = P_five - tfinal_vectors(tan_final,:)/5;

                P_three = acceleration_final(acc_final,:)/20 + 2*P_four - P_five;
                Curves{tan_zero + tan_final + acc_zero+ acc_final - 3} = zeros(101,2);
                pos = 1;
                for t = 0:0.01:1
                     Curves{curve_position}(pos,:) =...
                         (1-t)^5 *P_zero + 5*t*(1-t)^4*P_one +...
                          10* t^2 *(1-t)^3 *P_two + 10 * t^3*(1-t)^2 *P_three+...
                         +5*t^4*(1-t)*P_four + t^5*P_five;
                     pos = pos +1;
                end

                curve_position = curve_position+1;
            end
        end
    end
end

for m = 1:(curve_position-1)
    hold on
    plot(Curves{m}(:,1) , Curves{m}(:,2));

end



