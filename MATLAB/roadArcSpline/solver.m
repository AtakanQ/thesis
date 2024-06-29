syms k1 k2 k3 l3 sigma delta;

eqn = k2*k2 +k2*k1 -k2*k3 - k1*k1 - k3*l3*sigma +2*delta*sigma == 0
% eqn(2) = l2 + k2/sigma == 0
% eqn(3) = l1-k1/sigma == 0
S = solve(eqn,k2)

%% second case
clear
syms k1 k2 l2 k3 sigma delta

eqn = k1^2/(2*sigma) + k2*l2/2 + k3*l2/2 -k1*k2/(2*sigma) - k1*k3/(2*sigma) + k2*k3 /(2*sigma) - delta == 0
% eqn(2) = l2 + k2/sigma == 0
% eqn(3) = l1-k1/sigma == 0
S = solve(eqn,k2)