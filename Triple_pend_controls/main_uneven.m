%% main_uneven.m
% Triple Pendulum

function main_uneven

%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Set up events using odeset
options = odeset('Events', @(t,x) robot_events(t,x));

%% Simulate the robot forward in time
x_IC = params.x_IC';    % initial conditions
tnow = 0.0;             % starting time

% start with null matrices for holding results -- we'll be adding to these
% with each segment of the simulation
tsim = [];
xsim = [];
F_list = [];
te_sim = [];
ye_sim = [];

% create a place for constraint forces
F = [];

while tnow < params.sim.tfinal

    tspan = [tnow params.sim.tfinal];
    [tseg, xseg, te, ye, ie] = ode45(@robot_dynamics, tspan, x_IC, options);

    tsim = [tsim;tseg]; % augment simulation time
    xsim = [xsim;xseg]; % augment simulation states
    te_sim = [te_sim; te]; % augment event time
    ye_sim = [ye_sim; ye]; % augment location of events
    tnow = tsim(end); % updates time
    x_IC = xsim(end,:); % renew initial conditions

    % compute the constraint forces that were active
    [Fseg] = constraint_forces_uneven(tseg,xseg',params);
    F_list = [F_list,Fseg];

    % if simulation terminated before tfinal, determine which constraints
    % are still active, then continue integration
    if tseg(end) < params.sim.tfinal  % termination was triggered by an event
        switch params.sim.constraints
            case ['pumping'] % The robot is pumping up for Giant Swing
                params.sim.constraints = ['bar1'] % Finish giant swing on bar 1
            case ['bar1']  % The robot is on bar 1 prior to termination
                params.sim.constraints = ['flight']; % Robot releases from bar 1 and is in flight
            case ['flight'] % The robot is in flight prior to termination
                % solves for dq at time t+ right after impact
                dq_plus = collision(tseg(end), xseg(end,:)', params);
                % renew initial conditions
                x_IC(6:10) = dq_plus;
                params.sim.constraints = ['bar2']; % The robot catches bar 2
        end
    end
end

%%  Plot Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin with plot of ground reaction versus weight, to be sure we're
% pushing off and then leaving the ground
figure;
plot(tsim,F_list(1,:)+F_list(2,:),'b-','LineWidth',2);
hold on
weight = (params.model.dyn.top.m + params.model.dyn.mid.m + params.model.dyn.bot.m + ...
          params.model.dyn.motor1.m + params.model.dyn.motor2.m)*params.model.dyn.g*ones(1,length(tsim));
plot(tsim,weight,'r-','LineWidth',1);
ylabel('Ground Reaction vs Weight (N)')
xlabel('time (sec)')
hold off

% Now let's animate

% 1) deal with possible duplicate times in tsim:
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.viz.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(tsim,xsim,t_anim);
x_anim = x_anim'; % transpose so that x_anim is 10xN (N = number of timesteps)

% 4) resample the constraint forces-vs-time array:
F_anim = interp1(tsim,F_list',t_anim);
F_anim = F_anim'; % transpose so that F_anim is 10xN (N = number of timesteps)

animate_robot_uneven(x_anim(1:10,:),F_list,params,'trace_top_com',true,...
                                                  'trace_top_tip',true,...
                                                  'trace_mid_com',true,...
                                                  'trace_mid_tip',true,...
                                                  'trace_bot_com',true,...
                                                  'trace_bot_tip',true,...
                                                  'show_constraint_forces',true,...
                                                  'video',true);
fprintf('Done!\n');

%% BELOW HERE ARE THE NESTED FUNCTIONS, ROBOT_DYNAMICS AND ROBOT_EVENTS
%% THEY HAVE ACCESS TO ALL VARIABLES IN MAIN

%% robot_dynamics.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   Computes constraint forces (Fnow =
%   inv(A*Minv*A')*(A*Minv*(Q-H) + Adotqdot) and impacts (delta_dq =
%   -Minv*A'*inv(A*Minv*A')*(1+eps)*A*q_dot;)
%
%   Also computes the derivative of the state:
%       x_dot(1:5) = (I - A'*inv(A*A')*A)*x(6:10)
%       x_dot(6:10) = inv(M)*(Q - H - A'F)
%
% Inputs:
%   t: time (scalar)
%   x: the 10x1 state vector
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dx: derivative of state x with respect to time.

function [dx] = robot_dynamics(t,x)

% for convenience, define q_dot
dx = zeros(numel(x),1);
nq = numel(x)/2;    % assume that x = [q;q_dot];
q_dot = x(nq+1:2*nq);

% torque applied at shoulders
tau_shoulders = 0;
% torque applied at hips
tau_hips = 0;
% 5x1 Q column vector
Q = [0;
     0;
     0;
     tau_shoulders;
     tau_hips];

% H matrix
H = H_eom(x,params);

% 5x5 Mass Matrix
M = mass_matrix_uneven(x, params);

% 5x5 Coriolis Matrix
C = coriolis_matrix_uneven(x, params);

% 5x1 Grav Vector
G = grav_vector_uneven(x, params);

% 5x2 Selection Matrix
S = [0,0;
     0,0;
     0,0;
     1,0;
     0,1];

% 5x5 Inverse Mass Matrix
Minv = inv(M);

% function that returns A matrix and Hessians
[A_all,Hessian] = constraint_derivatives(x,params);

% function that returns drift vector field and control vector field
% [f_ss, g_ss] = state_space(x, Minv, C, G, S);

% solves for current robot state
xx = x(1);
yy = x(2);
th1 = x(3);
th2 = x(4);
th3 = x(5);
xx_dot = x(6);
yy_dot = x(7);
th1_dot = x(8);
th2_dot = x(9);
th3_dot = x(10);

% length of each link
l1 = params.model.geom.top.l;
l2 = params.model.geom.mid.l;
l3 = params.model.geom.bot.l;

% mass of each link
M1 = params.model.dyn.top.m;
M2 = params.model.dyn.mid.m;
M3 = params.model.dyn.bot.m;

% length of each link CoM
r1 = params.model.dyn.top.r_com;
r2 = params.model.dyn.mid.r_com;
r3 = params.model.dyn.bot.r_com;

% (x,y) location of top link CoM
com_l1x = xx + r1*sin(th1);
com_l1y = yy - r1*cos(th1);

% (x,y) location of mid link CoM
com_l2x = com_l1x + r2*sin(th1+th2);
com_l2y = com_l1y - r2*cos(th1+th2);

% (x,y) location of bot link CoM
com_l3x = com_l2x + r3*sin(th1+th2+th3);
com_l3y = com_l2y - r3*cos(th1+th2+th3);

% (x,y) location of robot CoM
com_x = (com_l1x*M1 + com_l2x*M2 + com_l3x*M3) / (M1+M2+M3);
com_y = (com_l1y*M1 + com_l2y*M2 + com_l3y*M3) / (M1+M2+M3);

% (x,y) location of top link tip
p1_x = xx + l1*sin(th1);
p1_y = yy - l1*cos(th1);

% (x,y) location of mid link tip
p2_x = p1_x + l2*sin(th1+th2);
p2_y = p1_y - l2*cos(th1+th2);

% (x,y) location of bot link tip
p3_x = p2_x + l3*sin(th1+th2+th3);
p3_y = p2_y - l3*cos(th1+th2+th3);

switch params.sim.constraints
    case ['pumping'] % robot is pumping
        % solves for constraint forces active when robot is pumping
        A = A_all([1,2],:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                    q_dot'*Hessian(:,:,2)*q_dot ];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [Fnow(1); Fnow(2)];
    case ['bar1'] % robot is on bar 1
        % solves for constraint forces active
        A = A_all([1,2],:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                    q_dot'*Hessian(:,:,2)*q_dot ];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [Fnow(1); Fnow(2)];
    case ['flight'] % robot is in flight
        % no constraint forces present
        dx(1:nq) = q_dot;
        dx(nq+1:2*nq) = Minv*(Q - H);
        F = [0;0];
    case ['bar2'] % robot is on bar 2
        % solves for constraint forces
        A = A_all([1,2],:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                    q_dot'*Hessian(:,:,2)*q_dot ];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [Fnow(1); Fnow(2)];
end
}%

end
%% end of robot_dynamics.m

%% Event function for ODE45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
%   Event function that is called when a constraint becomes inactive (or, in the future, active)

% Inputs:
%   t and x are required, but not used
%   F is shared with parent function

% Outputs:
%   value
%   isterminal
%   direction
function [value,isterminal,direction] = robot_events(t,x)
    % solves for current robot state
    xx = x(1);
    yy = x(2);
    th1 = x(3);
    th2 = x(4);
    th3 = x(5);

    % length of each link
    l1 = params.model.geom.top.l;
    l2 = params.model.geom.mid.l;
    l3 = params.model.geom.bot.l;

    % (x,y) location of top link tip
    p1_x = xx + l1*sin(th1);
    p1_y = yy - l1*cos(th1);

    % (x,y) location of mid link tip
    p2_x = p1_x + l2*sin(th1+th2);
    p2_y = p1_y - l2*cos(th1+th2);

    % (x,y) location of bot link tip
    p3_x = p2_x + l3*sin(th1+th2+th3);
    p3_y = p2_y - l3*cos(th1+th2+th3);

    switch params.sim.constraints
        case ['pumping'] % robot is pumping up to Giant Swing
            value = 1;
            isterminal = 0;
            direction = 0;
%             % if the robot is along positive y-axis
%             value = [p1_x - params.sim.bar1.x;
%                      p1_y - (params.sim.bar1.y + l1);
%                      p2_x - params.sim.bar1.x;
%                      p2_y - (params.sim.bar1.y + l1 + l2);
%                      p3_x - params.sim.bar1.x;
%                      p3_y - (params.sim.bar1.y + l1 + l2 + l3)];
%             % terminate integration (trigger event)
%             isterminal = 1;
%             % positive or negative direction
%             direction = 0;
        case ['bar1'] % robot is on bar 1
            % if the x location of second pendulum = x location of bar 1
            value = p2_x - params.sim.bar1.x;
            % terminate integration (trigger event)
            isterminal = 1;
            % positive or negative direction
            direction = 0;
        case ['flight'] % robot is in flight
            % if the x location of the gripper = x location of bar 2
            value = abs(xx) - params.sim.bar2.x;
            % terminate integration (trigger event)
            isterminal = 1;
            % positive or negative direction
            direction = 0;
        case ['bar2'] % robot is on bar 2
            % always true
            value = 1;
            % continue integration (no event triggered)
            isterminal = 0;
            % positive or negative direction
            direction = 0;
    end
end
%% end of robot_events.m
end
%% End of main.m
