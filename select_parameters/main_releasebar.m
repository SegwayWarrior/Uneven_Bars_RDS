%% main_releasebar.m 
% run this function to see the robot
%% Initialize environment
function [success] = main_releasebar
clear;
close all;
clc;

init_env();
catch_range = 0.1;
success = 0;

%% Initialize parameters
params = init_params_release;

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
    
plot_robot_release(x_IC(1:5),params,'new_fig',false);

%% Simulate the robot forward in time with no control input
tspan_passive = 0:params.sim.dt:1.5;

[tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics_release(...
    t,x,0,params,'controller','passive'),...
    tspan_passive, x_IC');

catch_time = size(xsim_passive,1);
for ii = 1 : size(xsim_passive,1)
    if ((xsim_passive(ii,1)- params.model.higher.top.x)^2 + (xsim_passive(ii,2)- params.model.higher.top.y)^2) < catch_range
        catch_time = ii;
        break;
    end
end
if (catch_time < size(xsim_passive,1))
    success = 1;
end
xsim_passive = xsim_passive';



catch_state = xsim_passive(:,catch_time);
save('savecatchstate','catch_state');
pause(1); 
animate_robot_release(xsim_passive(1:5,1:catch_time),params,'trace_motor1',true,...
   'trace_motor2',true,'trace_top',true,'video',true);

end