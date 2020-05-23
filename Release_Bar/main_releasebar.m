%% main_releasebar.m 
% run this function to see the robot
%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Visualize the robot in its initial state
x_IC = [params.sim.ICs.x_top;
        params.sim.ICs.y_top;
        params.sim.ICs.theta_1;
        params.sim.ICs.theta_2;
        params.sim.ICs.theta_3;
        params.sim.ICs.dx_top;
        params.sim.ICs.dy_top;
        params.sim.ICs.dtheta_1;
        params.sim.ICs.dtheta_2;
        params.sim.ICs.dtheta_3];
    
plot_robot(x_IC(1:5),params,'new_fig',false);

%% Simulate the robot forward in time with no control input
tspan_passive = 0:params.sim.dt:3;

[tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics(...
    t,x,0,params,'controller','passive'),...
    tspan_passive, x_IC');
xsim_passive = xsim_passive';

pause(1); 
animate_robot(xsim_passive(1:5,:),params,'trace_motor1',true,...
   'trace_motor2',true,'trace_top',true,'video',true);