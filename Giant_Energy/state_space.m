%% Generate (nonlinear) state-space model from manipulator equation

function [f_ss, g_ss] = state_space(x, Minv, C, G, S)
dq = [x(6); 
      x(7); 
      x(8); 
      x(9); 
      x(10)];
 
temp_drift = -Minv*(C*dq + G); % it runs about 1 hour
% matfile_drift = matfile('savetemp_drift.mat');
% temp_drift = matfile_drift.temp_drift;

temp_ctrl = Minv*S; 
% matfile_ctrl = matfile('savetemp_ctrl.mat');
% temp_ctrl = matfile_ctrl.temp_ctrl;

f_ss = zeros(numel(x),1);
f_gg = zeros(numel(x),2);

% Build state-space representation:
for i = 1:numel(dq)
    f_ss(i) = dq(i);
    g_ss(i,:) = [0,0];
    f_ss(i+numel(dq)) = temp_drift(i);
    g_ss(i+numel(dq),:) = temp_ctrl(i,:);
end

end