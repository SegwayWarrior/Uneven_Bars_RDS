%% main_uneven.m
% Triple Pendulum

function main_uneven

%% Initialize environment
clear;
close all;
clc;

init_env();

persistent check
if isempty(check)
    check = 1;
end
check = 1;

persistent check2
if isempty(check2)
    check2 = 1;
end
check2 = 1;

persistent check3
if isempty(check3)
    check3 = 1;
end
check3 = 1;

%% Initialize parameters
params = init_params;

%% Set up events using odeset
options = odeset('Events',@robot_events);

%% Set the initial equilibrium pose of the robot
x_eq = zeros(10,1);
% Set to upright equalibrium
x_eq(2) = 0;  % first bar's y position
x_eq(3) = 3.141; % th1 at upright

M_eq = mass_matrix_uneven(x_eq,params);    % mass matrix at equilibrium

%% Simulate the robot forward in time
x_IC = params.x_IC';    % initial conditions
% x_IC = x_eq;
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

% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.sim.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(tsim,xsim,t_anim);
x_anim = x_anim'; % transpose so that xsim is 10xN (N = number of timesteps)

% 4) resample the constraint forces-vs-time array:
F_anim = interp1(tsim,F_list',t_anim);
F_anim = F_anim';

animate_robot_uneven(x_anim(1:10,:),F_list,params,'trace_top_com',false,...
                                                  'trace_top_tip',false,...
                                                  'trace_mid_com',false,...
                                                  'trace_mid_tip',false,...
                                                  'trace_bot_com',false,...
                                                  'trace_bot_tip',false,...
                                                  'show_constraint_forces',false,'video',true);
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

% solve for control inputs at this instant
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

l1 = params.model.geom.top.l;
l2 = params.model.geom.mid.l;
l3 = params.model.geom.bot.l;

M1 = params.model.dyn.top.m;      % mass of the top link
M2 = params.model.dyn.mid.m;      % mass of the mid link
M3 = params.model.dyn.bot.m;      % mass of the bot link

com_l1x = xx + l1*sin(th1)/2;
com_l1y = yy - l1*cos(th1)/2;

com_l2x = xx + l1*sin(th1) + l2*sin(th1+th2)/2;
com_l2y = yy - l1*cos(th1) - l2*cos(th1+th2)/2;

com_l3x = xx + l1*sin(th1) + l2*sin(th1+th2) + l3*sin(th1+th2+th3)/2;
com_l3y = yy - l1*cos(th1) - l2*cos(th1+th2) - l3*cos(th1+th2+th3)/2;

com_x = (com_l1x*M1 + com_l2x*M2 + com_l3x*M3) / (M1+M2+M3);
com_y = (com_l1y*M1 + com_l2y*M2 + com_l3y*M3) / (M1+M2+M3);
% 
% % com_list(end+1) = com_y;
% % assignin('base','com_list',com_list)
persistent com_y_pre1
if isempty(com_y_pre1)
    com_y_pre1 = com_y;
end
% persistent com_y_pre2
% if isempty(com_y_pre2)
%     com_y_pre2 = com_y;
% end
% persistent com_y_pre3
% if isempty(com_y_pre3)
%     com_y_pre3 = com_y;
% end
% persistent com_y_pre4
% if isempty(com_y_pre4)
%     com_y_pre4 = com_y;
% end
% persistent com_y_pre5
% if isempty(com_y_pre5)
%     com_y_pre5 = com_y;
% end
% persistent com_y_pre_avg
% if isempty(com_y_pre_avg)
%     com_y_pre_avg = com_y;
% end
persistent com_x_pre1
if isempty(com_x_pre1)
    com_x_pre1 = com_x;
end
% persistent com_x_pre2
% if isempty(com_x_pre2)
%     com_x_pre2 = com_x;
% end
% persistent com_x_pre3
% if isempty(com_x_pre3)
%     com_x_pre3 = com_x;
% end
% persistent com_x_pre4
% if isempty(com_x_pre4)
%     com_x_pre4 = com_x;
% end
% persistent com_x_pre5
% if isempty(com_x_pre5)
%     com_x_pre5 = com_x;
% end
% persistent com_x_pre_avg
% if isempty(com_x_pre_avg)
%     com_x_pre_avg = com_x;
% end
% persistent counteri
% if isempty(counteri)
%     counteri = [];
% end

com_x_change = com_x - com_x_pre1;
com_y_change = com_y - com_y_pre1;
% 
% % counteri(end+1) = com_y_change;
% % assignin('base','counteri',counteri);
% 
% tau_shoulders = 0;
% tau_hips = 0;
% pi = 3.141;
% 
% if (com_x >= com_x_pre_avg) % moving to right and low
% %     if (com_y <=  com_y_pre) % falling
% %         th2des = 0;
% %         th3des = 0;
% %     else % rising
%         th2des = pi/3;
%         th3des = pi/3;
% %     end
%      
% else
% %     if (com_y <=  com_y_pre_avg)% falling
% %         th2des = 0;
% %         th3des = 0;
% %     else % rising
%         th2des = -pi/3;
%         th3des = -pi/3;
% %     end
%     
% end

% if (com_x < 1.5) && (com_x > -1.5) && abs(com_x - com_x_pre_avg < 1)
%     th2des = pi/2;
%     th3des = pi/2;
% end
    

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

% setting up energy controller
M_uu = M(3,3);
M_ua = M(3,4:5);
M_au = M(4:5,3);
M_aa = M(4:5,4:5);

C_uu = C(3,3);
C_ua = C(3,4:5);
C_au = C(4:5,3);
C_aa = C(4:5,4:5);

G_u = G(3);
G_a = G(4:5);

qu_dot = x(8);
qa_dot = x(9:10); % th2dot & th3dot
qa = x(4:5);

% set up qa_ddot with energy shaping control
kinEner = kinetic_energy(x,params);
potEner = potential_energy(x,params);
totEner = total_energy(x,params);

potMax1 = 294.3426;  % max potential energy on bar 1 (0,6)
potMin1 = 176.5817;  % min

desEner = 800; % gets robot to about 10 rad/sec
% desEner = potMin1;

% % can only do a giant counter clockwise
% if (th1_dot<-0.1) && (sin(th1)<0.1) &&(cos(th1)<0.1)
%     desEner = -500; % potential energy at bar height
% end
% desEner;

Eerr = totEner - desEner;

k12const = 70;
k3const = 550;

k_21 = k12const;
k_22 = k12const;
k_23 = k3const;
k_31 = k12const;
k_32 = k12const;
k_33 = k3const;
k_mat = [k_21 0 k_22 0; 0 k_31 0 k_32];
    
q1_dot = x(8); 
q2_dot = x(9);

Eerr_mat = [k_23*Eerr*q1_dot; k_33*Eerr*q2_dot];

qa_ddot = -1*k_mat*[qa; qa_dot] + Eerr_mat;

torq_vect = (M_aa-M_au*inv(M_uu)*M_ua)*qa_ddot - M_au*inv(M_uu)*(C_uu*qu_dot+C_ua*qa_dot+G_u)+C_au*qu_dot + C_aa*qa_dot + G_a;

tau_shoulders = torq_vect(1);
tau_hips = torq_vect(2);
    

% PD controller
kp2 = 100;
kp3 = 100;
kd2 = 10;
kd3 = 10;
th2des = 0;
th3des = 0;

persistent th2pre
if isempty(th2pre)
    th2pre = th2;
end
persistent th3pre
if isempty(th3pre)
    th3pre = th3;
end
persistent th2errorpre
if isempty(th2errorpre)
    th2errorpre = 0;
end
persistent th3errorpre
if isempty(th3errorpre)
    th3errorpre = 0;
end

% Start with a curl
th2des = pi/2;
th3des = pi/2;
th2error = th2des - th2;
th3error = th3des - th3;


% if( totEner < 190)
%     tau_shoulders = torq_lim;
%     tau_hips = torq_lim;
% elseif( totEner < 210) && ((com_x -com_x_pre1) >0)     %(potMin1 + (potMax1-potMin1)/6))
%     tau_shoulders = torq_lim;
%     tau_hips = torq_lim;
% end
% if( totEner < 210) && ((com_x -com_x_pre1) <0)     
%     tau_shoulders = -torq_lim;
%     tau_hips = -torq_lim;
% end

% % Falling PD controller
% th2des = -th1;
% th3des = th2des;
% th2error = th2des - th2;
% th3error = th3des - th3;
% if (com_y < com_y_pre1)
%     tau_shoulders = kp2*th2error + kd2*(th2error - th2errorpre);
%     tau_hips = kp3*th3error + kd3*(th3error - th3errorpre);
% end

th2errorpre = th2error;
th3errorpre = th3error;


% set torque limits
torq_lim = 1.3;
if (tau_shoulders > torq_lim)
    tau_shoulders = torq_lim;
end
if (tau_shoulders < -torq_lim)
    tau_shoulders = -torq_lim;
end
if (tau_hips > torq_lim)
    tau_hips = torq_lim;
end
if (tau_hips < -torq_lim)
    tau_hips = -torq_lim;
end


% Internal Collisions - partially reverse rotational velocity if past 90deg
if (th2 > pi/2) && (th2_dot >0)
    x(9) = 0;
elseif (th2 < -pi/2) && (th2_dot <0)
    x(9) = 0;
end
if (th3 > pi/2) && (th3_dot >0)
    x(10) = 0;
elseif (th3 < -pi/2) && (th3_dot <0)
    x(10) = 0;
end

th1pre = th1;
  

% bar release
Q = [0;
     0;
     0;
     tau_shoulders;
     tau_hips];
 
%  Q = [0;
%      0;
%      0;
%      torq_lim;
%      torq_lim];
 
% on bar 

A = A_all([1,2],:);
Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
            q_dot'*Hessian(:,:,2)*q_dot];
Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
F = [Fnow(1); Fnow(2)];

% release
% check
% com_x
xx
totEner;
% if (potEner > 0) && (com_x > xx)
%     Q = [0;
%      0;
%      0;
%      torq_lim;
%      torq_lim];
% end
% if (potEner > 20) && (com_x < xx) 
% %     s = ['false', 'false'];
%     check = 2;  
% end
% if (check == 2) % curl up for  ready for release
%     tau_shoulders = kp2*th2error + kd2*(th2error - th2errorpre);
%     tau_hips = kp3*th3error + kd3*(th3error - th3errorpre);
%     
%     if (tau_shoulders > torq_lim)
%     tau_shoulders = torq_lim;
%     end
%     if (tau_shoulders < -torq_lim)
%         tau_shoulders = -torq_lim;
%     end
%     if (tau_hips > torq_lim)
%         tau_hips = torq_lim;
%     end
%     if (tau_hips < -torq_lim)
%         tau_hips = -torq_lim;
%     end
%     Q = [0;
%      0;
%      0;
%      tau_shoulders;
%      tau_hips];
%     check2 = 3;
% end
if (potEner > 40) &  (com_x < 0)
    check2 = 2;
end
    
if (check2 ==2) && (potEner > -15) && (com_x > 0) % && (chck2 == 3)
    check = 1;
    check3 = 4;
    dx(1:nq) = q_dot;
     
     Q = [0;
     0;
     0;
     0;
     0];
    dx(1:nq) = q_dot;
    dx(nq+1:2*nq) = Minv*(Q - H);
    F = [0;0];
    if (th2 > pi/2) && (th2_dot >0)
        dx(4) = 0;
    elseif (th2 < -pi/2) && (th2_dot <0)
        dx(4) = 0;
    end
    if (th3 > pi/2) && (th3_dot >0)
        dx(5) = 0;
    elseif (th3 < -pi/2) && (th3_dot <0)
        dx(5) = 0;
    end

    dx(nq+1:2*nq) = Minv*(Q - H);
    F = [0;0];
end
if (check3 == 4)
    check2 = 1;
    dx(1:nq) = q_dot;
     
     Q = [0;
     0;
     0;
     torq_lim;
     torq_lim];
    dx(1:nq) = q_dot;
    dx(nq+1:2*nq) = Minv*(Q - H);
    F = [0;0];
    if (th2 > pi/2) && (th2_dot >0)
        dx(4) = 0;
    elseif (th2 < -pi/2) && (th2_dot <0)
        dx(4) = 0;
    end
    if (th3 > pi/2) && (th3_dot >0)
        dx(5) = 0;
    elseif (th3 < -pi/2) && (th3_dot <0)
        dx(5) = 0;
    end

    dx(nq+1:2*nq) = Minv*(Q - H);
    F = [0;0];
end

if (xx > 2.6) && (xx < 3.2) && (yy > 0.9) && (yy < 1.5)
    check3 = 1;
%     xx_des = 2.9;
%     yy_des = 1.2;
%     kp_gripper = .1;
   
    A = A_all([1,2],:);
    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                q_dot'*Hessian(:,:,2)*q_dot];
    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
    
    % fix gripper possition to match high bar
    if(xx < 2.9)
        dx(1) = 1;
    elseif(xx > 2.9)
        dx(1) = -1;
    else
        dx(1) = 0;
    end
    
    if(yy < 1.2)
        dx(2) = 1;
    elseif(yy < 1.2)
        dx(2) = -1;
    else
        dx(2) = 0;
    end
%     dx(2)= kp_gripper*(yy - yy_des);
    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
    F = [Fnow(1); Fnow(2)];
end

delta_t = params.sim.dt;

% persistent kinEnerMax
% if isempty(kinEnerMax)
%     kinEnerMax = 0;
% end
% persistent potEnerMax
% if isempty(potEnerMax)
%     potEnerMax = 0;
% end
persistent potEnerMin
if isempty(potEnerMin)
    potEnerMin = 800;
end
% persistent totEnerMax
% if isempty(totEnerMax)
%     totEnerMax = 0;
% end

% potEner = (M1+M2+M3)*(9.8)*(com_y);
% kinEner = 0.5*(M1+M2+M3)*(((com_x-com_x_pre_avg)/delta_t)^2 + ((com_y-com_y_pre_avg)/delta_t)^2);

% if (kinEner > kinEnerMax)
%     kinEnerMax = kinEner;
% end
% if (potEner > potEnerMax)
%     potEnerMax = potEner;
% end
if (potEner < potEnerMin)
    potEnerMin = potEner;
end
potEnerMin;
% if (totEner > totEnerMax)
%     totEnerMax = totEner;
% end

% kinEnerMax
% potEnerMax
% potEnerMin
% totEnerMax;

% com_x_pre1 = com_x;
% com_y_pre1 = com_y;
% com_x_pre2 = com_x_pre1;
% com_y_pre2 = com_y_pre1;
% com_x_pre3 = com_x_pre2;
% com_y_pre3 = com_y_pre2;
% com_x_pre4 = com_x_pre3;
% com_y_pre4 = com_y_pre3;
% com_x_pre5 = com_x_pre4;
% com_y_pre5 = com_y_pre4;
% com_x_pre_avg = (com_x_pre1+com_x_pre2+com_x_pre3+com_x_pre4+com_x_pre5)/5;
% com_y_pre_avg = (com_y_pre1+com_y_pre2+com_y_pre3+com_y_pre4+com_y_pre5)/5;


% TRYING TO RELEASE FROM BAR
% xx = x(1);
% yy = x(2);
% th1 = x(3);
% th2 = x(4);
% th3 = x(5);
% 
% l1 = params.model.geom.top.l;
% l2 = params.model.geom.mid.l;
% l3 = params.model.geom.bot.l;
% 
% p1_x = xx + l1*sin(th1);
% p1_y = yy - l1*cos(th1);
% 
% p2_x = p1_x + l2*sin(th1+th2);
% p2_y = p1_y - l2*cos(th1+th2);
% 
% p3_x = p2_x + l3*sin(th1+th2+th3);
% p3_y = p2_y - l3*cos(th1+th2+th3);

% if on bar
%     s = ['true', 'true'];

% elseif (params.sim.bar1.x < p1_x < params.sim.bar2.x) &&...
%        (params.sim.bar1.x < p2_x < params.sim.bar2.x) &&...
%        (params.sim.bar1.x < p3_x < params.sim.bar2.x)

% elseif (params.sim.bar2.x < p1_x) &&...
%        (params.sim.bar2.x < p2_x) &&...
%        (params.sim.bar2.x < p3_x)
%     s = ['true', 'true'];
%     A = A_all([1,2],:);
%     Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
%                 q_dot'*Hessian(:,:,2)*q_dot];
%     Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
%     dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
%     dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
%     F = [Fnow(1); Fnow(2)];
% end
% }%

end
%% end of robot_dynamics.m


%% Event function for ODE45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
%   Event function that is called when a constraint becomes inactive (or, in the future, active)

% Inputs:
%   t and x are required, but not used
%   F is shared with parent function
%
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
            isterminal = 0;
            direction = 0;
        case ['true','true'] % robot is on the bar
            value = [F(1); F(2)];
            isterminal = ones(2,1);
            direction = zeros(2,1);
    end

end
%% end of robot_events.m
end
%% End of main.m
