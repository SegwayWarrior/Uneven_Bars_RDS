%% init_params.m

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
    
    params.viz.colors.top = [1 0.4 0.8];
    params.viz.colors.mid = [1 0.6 0.8];
    params.viz.colors.bot = [1 0.8 0.8];
    
    params.viz.colors.tracer.top = [0.5 0 0.5];
    params.viz.colors.tracer.mid = [0.7 0 0.7];
    params.viz.colors.tracer.bot = [0.9 0 0.9];
    
    params.viz.axis_lims = [-10,14,-8,10];
    params.viz.energy_lims = [-100,100,-100,100];
        
    % first bar location
    params.sim.bar1.x = 0;
    params.sim.bar1.y = 6;
    
    % second bar location
    params.sim.bar2.x = 4.0012;
    params.sim.bar2.y = 0.8998;
    
    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.x = params.sim.bar1.x;        % initial x position
    params.sim.ICs.y = params.sim.bar1.y;        % initial y position
    params.sim.ICs.theta_1 = -3*pi/4;  % initial theta_1 position
    params.sim.ICs.theta_2 = 0;  % initial theta_2 position
    params.sim.ICs.theta_3 = 0;  % initial theta_3 position
    
    params.sim.ICs.dx = 0;       % initial x velocity
    params.sim.ICs.dy = 0;       % initial y velocity
    params.sim.ICs.dtheta_1 = 0; % initial theta_1 velocity
    params.sim.ICs.dtheta_2 = 0; % initial theta_2 velocity
    params.sim.ICs.dtheta_3 = 0; % initial theta_3 velocity
    
    params.x_IC = [params.sim.ICs.x;
                   params.sim.ICs.y;
                   params.sim.ICs.theta_1;
                   params.sim.ICs.theta_2;
                   params.sim.ICs.theta_3;
                   params.sim.ICs.dx;
                   params.sim.ICs.dy;
                   params.sim.ICs.dtheta_1;
                   params.sim.ICs.dtheta_2;
                   params.sim.ICs.dtheta_3];
               
    params.sim.dt = 0.01;  % simulation timestep
    params.viz.dt = 0.5;  % visualization timestep
    params.sim.tfinal = 5; % simulation final time
    
    params.motor1.peaktorque = 1.0; % Nm assumes Maxon EC40 and 3.3x gear ratio
    params.motor2.peaktorque = 1.0; % Nm assumes Maxon EC40 and 3.3x gear ratio
    
    % variables related to the constraints
    params.sim.constraints.number = 2;
    % list of constraints: [x, y]   1 if active; 0 if inactive
    params.sim.constraints = ['true','true'];   % initially, both constraints are active
end

%     % motor torques over time:
%         % create a timeline for use here
%         tfinal = params.sim.tfinal;
%         dt = params.sim.dt;
%     % push the spine cw against the spring
%     params.motor.spine.time = 0:dt:tfinal;
%     params.motor.spine.torque = 0*params.motor.spine.peaktorque*ones(1,length(params.motor.spine.time)); % push the spine cw against the spring
%     params.motor.spine.torque(1) = 0;  % make the initial torque zero
%     % push the body upward as hard as possible then brake
%     params.motor.body.time1 = 0:dt:.183;
%     params.motor.body.torque1 = params.motor.body.peaktorque*ones(1,length(params.motor.body.time1)); 
%     params.motor.body.torque1(1) = params.model.dyn.body.m*params.model.dyn.g*params.model.geom.body.r;  % make the initial torque enough to hold up body
%     params.motor.body.time2 = .183+dt:dt:.25;
%     params.motor.body.torque2 = -2*params.motor.body.peaktorque*ones(1,length(params.motor.body.time2));
%     params.motor.body.time3 = .25+dt:dt:tfinal;
%     params.motor.body.torque3 = .2*params.model.dyn.body.m*params.model.dyn.g*params.model.geom.body.r*ones(1,length(params.motor.body.time3));
%     params.motor.body.time = horzcat(params.motor.body.time1,params.motor.body.time2,params.motor.body.time3);
%     params.motor.body.torque = horzcat(params.motor.body.torque1,params.motor.body.torque2,params.motor.body.torque3);                                  