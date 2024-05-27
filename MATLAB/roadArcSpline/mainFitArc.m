clear


%% Try to fit clothoid
% load('bezierData.mat')
load("all_clothoids.mat")
xError =  +0.2; %meters in x direction
yError = -0.01; %mteres is y direction
headingError = deg2rad(5); %radians
curvatureError = -0.01;

clothoid = all_clothoids{1}(158);
[clothoidArray] = fitArcSpline([clothoid.allX(1)+xError clothoid.allY(1)+yError],...
    clothoid.init_tan+headingError,clothoid.init_curv + curvatureError,clothoid);