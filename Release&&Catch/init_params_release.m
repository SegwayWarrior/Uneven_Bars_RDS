% init_params.m
function params = init_params_release
%% parameters that appear in the dynamics:
    % mass of all links and motors
    params.model.dyn.top.m = 1;      % mass of the top link
    params.model.dyn.mid.m = 1;      % mass of the mid link
    params.model.dyn.bot.m = 1;      % mass of the bot link
    params.model.dyn.motor1.m = 0.5; % mass of motor 1
    params.model.dyn.motor2.m = 0.5; % mass of motor 2
% moment of inertia of Link1,2,3
    params.model.dyn.top.I = 0.01; % moment of inertia of top link
    params.model.dyn.mid.I = 0.01; % moment of inertia of mid link
    params.model.dyn.bot.I = 0.01; % moment of inertia of bot link
% length of Link1,2,3
    params.model.dyn.top.l = 1; 
    params.model.dyn.mid.l = 1; 
    params.model.dyn.bot.l = 1; 
% CoM to tip links length
    params.model.dyn.top.r_com = 0.5; % radial loc. of top link CoM
    params.model.dyn.mid.r_com = 0.5; % radial loc. of mid link CoM
    params.model.dyn.bot.r_com = 0.5; % radial loc. of bot link CoM
% acceleration due to gravity
    params.model.dyn.g = 9.81;
% damping coefficient
    params.model.dyn.b1 = 0;      % damping 1
    params.model.dyn.b2 = 0;      % damping 2
    params.model.dyn.b3 = 0.2;      % damping 3
    params.model.dyn.b4 = 0.2;      % damping 1
    params.model.dyn.b5 = 0.2;      % damping 2
    
%% parameters that help with visualizing the robot:

    params.model.geom.top.l = 1; % length of top link
    params.model.geom.mid.l = 1; % length of mid link
    params.model.geom.bot.l = 1; % length of bot link
    
    params.model.geom.top.r_com = 0.5;
    params.model.geom.mid.r_com = 0.5;
    params.model.geom.bot.r_com = 0.5;
    % link color
    params.viz.colors.top = [0.8500, 0.3250, 0.0980];
    params.viz.colors.mid = [0.4660, 0.6740, 0.1880];
    params.viz.colors.bot = [0.4940, 0.1840, 0.5560];
    % motor color
    params.viz.colors.motor.edge1 = [0, 0.4470, 0.7410];
    params.viz.colors.motor.edge2 = [0.3010, 0.7450, 0.9330];
    params.viz.colors.motor.face1 = [0.1, 0.5470, 0.8410];
    params.viz.colors.motor.face2 = [0.4010, 0.8450, 0.9330];
    % trace color
    params.viz.colors.tracers.top = [0.1, 0.4470, 0.7410];
    params.viz.colors.tracers.mid = [0.4010, 0.7450, 0.9330];
    params.viz.colors.tracers.bot = [1, 0, 0];
    % motor model
    params.model.geom.motor.mr1 = 100;   % radius of motor1, m is motor
    params.model.geom.motor.mr2 = 100;   % mr is for different with r1
    params.viz.axis_lims = [-5,15,-5,15];
    
 %% parameters related to simulating (integrating) the dynamics forward in time:
 % initialize parameters:
    params.sim.ICs.x_top = 0;
    params.sim.ICs.y_top = 0;
    params.sim.ICs.theta_1 = pi/3;      % initial theta_1 position
    params.sim.ICs.theta_2 = pi/4;      % initial theta_2 position
    params.sim.ICs.theta_3 = 0;      % initial theta_3 position
    
    params.sim.ICs.dx_top = 0;
    params.sim.ICs.dy_top = 0;
    params.sim.ICs.dtheta_1 = 2.5;     % initial theta_1 velocity
    params.sim.ICs.dtheta_2 = 1;     % initial theta_2 velocity
    params.sim.ICs.dtheta_3 = 1;     % initial theta_3 velocity
    
    params.sim.dt = 0.05;           % simulation timestep



%% add the bar position
    params.model.lower.top.x = 0; % lower bar location
    params.model.lower.top.y = 0;
    params.model.lower.bot.x = 0;
    params.model.lower.bot.y = -5;
    params.model.higher.top.x = 10; % higher bar location
    params.model.higher.top.y = 5;
    params.model.higher.bot.x = 10;
    params.model.higher.bot.y = -5;
end