%% init_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%   
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements

function params = init_params
    % parameters that appear in the dynamics:
    params.model.dyn.top.m = 1;      % mass of the top link
    params.model.dyn.mid.m = 1;      % mass of the mid link
    params.model.dyn.bot.m = 1;      % mass of the bot link
    params.model.dyn.motor1.m = 0.5; % mass of motor 1
    params.model.dyn.motor2.m = 0.5; % mass of motor 2
    
    params.model.dyn.top.I = 0.01; % moment of inertia of top link
    params.model.dyn.mid.I = 0.01; % moment of inertia of mid link
    params.model.dyn.bot.I = 0.01; % moment of inertia of bot link
    
    params.model.dyn.top.r_com = 0.5; % radial loc. of top link CoM
    params.model.dyn.mid.r_com = 0.5; % radial loc. of mid link CoM
    params.model.dyn.bot.r_com = 0.5; % radial loc. of bot link CoM
    
    params.model.dyn.g = 9.81;      % acceleration due to gravity
    params.model.dyn.b1 = 0.2;      % damping 1
    params.model.dyn.b2 = 0.2;      % damping 2
    params.model.dyn.b3 = 0.2;      % damping 3
    
    % parameters that help with visualizing the robot:
    params.model.geom.top.l = 1; % length of top link
    params.model.geom.mid.l = 1; % length of mid link
    params.model.geom.bot.l = 1; % length of bot link
    
    params.model.geom.top.r_com = 0.5;
    params.model.geom.mid.r_com = 0.5;
    params.model.geom.bot.r_com = 0.5;
    
    params.viz.colors.top = [1 1 1];
    params.viz.colors.mid = [1 1 1];
    params.viz.colors.bot = [1 1 1];
    
    params.viz.colors.top = [1 0.4 0.8];
    params.viz.colors.mid = [1 0.6 0.8];
    params.viz.colors.bot = [1 0.8 0.8];
    
    params.viz.axis_lims = [-4,4,-4,4];
    params.viz.energy_lims = [-100,100,-100,100];
    
    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.theta_1 = pi/2;   % initial theta_1 position
    params.sim.ICs.theta_2 = 0;      % initial theta_2 position
    params.sim.ICs.theta_3 = 0;      % initial theta_3 position
    
    params.sim.ICs.dtheta_1 = 0;     % initial theta_1 velocity
    params.sim.ICs.dtheta_2 = 0;     % initial theta_2 velocity
    params.sim.ICs.dtheta_3 = 0;     % initial theta_3 velocity
    
    params.sim.dt = 0.05;           % simulation timestep
end