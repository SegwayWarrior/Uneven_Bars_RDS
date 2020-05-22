%% Robot_dynamics.m

function [dx,F] = robot_dynamics(~,x)

params = init_params;
% for convenience, define q_dot
dx = zeros(numel(x),1);
nq = numel(x)/2;    % assume that x = [q;q_dot];
q_dot = x(nq+1:2*nq);

% if both feet are on ground and body hasn't hit top, then apply max torque
if params.sim.constraints.uni(1) == 1 && params.sim.constraints.uni(2) == 1
    Q = zeros(5,1);
else  % otherwise, apply no torque at all
    Q = zeros(5,1);
end