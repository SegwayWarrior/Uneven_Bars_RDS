%% main.m

function main_catchbar

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
tspan_passive = 0:params.sim.dt:8;
[tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics(...
    t,x,0,params,'controller','passive'),...
    tspan_passive, x_IC');

xsim_passive = xsim_passive'; % required by animate_robot.m

pause(1); 
animate_robot(xsim_passive(1:3,:),params,'trace_motor1',true,...
   'trace_motor2',true,'trace_top',true,'video',true);
end
