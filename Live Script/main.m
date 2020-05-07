%% main.m

function main

%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Visualize the robot in its initial state
x_IC = [params.sim.ICs.theta_1;
        params.sim.ICs.theta_2;
        params.sim.ICs.theta_3;
        params.sim.ICs.dtheta_1;
        params.sim.ICs.dtheta_2;
        params.sim.ICs.dtheta_3];
 
plot_robot(x_IC(1:3),params,'new_fig',false);

%% Simulate the robot forward in time with no control input
tspan_passive = 0:params.sim.dt:20;
[tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics(...
    t,x,0,params,'controller','passive'),...
    tspan_passive, x_IC');

xsim_passive = xsim_passive'; % required by animate_robot.m

figure;
% configuration vs. time
subplot(2,1,1), plot(tsim_passive,xsim_passive(1,:),'b:',...
                     tsim_passive,xsim_passive(2,:),'r:',...
                     tsim_passive,xsim_passive(3,:),'g:','LineWidth',2);
% velocities vs. time
subplot(2,1,2), plot(tsim_passive,xsim_passive(4,:),'b:',...
                     tsim_passive,xsim_passive(5,:),'r:',...
                     tsim_passive,xsim_passive(6,:),'g:','LineWidth',2);


pause(1); % helps prevent animation from showing up on the wrong figure
animate_robot(xsim_passive(1:3,:),params,...
    'trace_top_com', true, 'trace_top_tip', true,...
    'trace_mid_com', true, 'trace_mid_tip', true,...
    'trace_bot_com', true, 'trace_bot_tip', true, 'video',true);
fprintf('Done passive simulation.\n');
end
