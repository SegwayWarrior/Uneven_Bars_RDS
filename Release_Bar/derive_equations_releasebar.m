%% derive_equations_releasebar.m
% this is derive_equations.m of releasing from the bar.
% there are five generalized variables: (x_top,y_top)is the position of top link;
% theta_1, theta_2, theta_3 is the angle of top, middle, bottom linkl

%function derive_equations_releasebar
clear;
close all;
clc;

%% Define parameters

% Generalized coordinates:
% x_top: horizontal coordinate of toplink
% y_top: vertical coordinate of top link 
% theta_1: angle of top link, ccw+ wrt x-axis
% theta_2: angle of mid link, ccw+ wrt top link
% theta_3: angle of bot link, ccw+ wrt mid link
syms x_top y_top theta_1 theta_2 theta_3 real
q = [x_top; y_top; theta_1; theta_2; theta_3];

% velocities:
syms dx_top dy_top dtheta_1 dtheta_2 dtheta_3 real
dq = [dx_top; dy_top; dtheta_1; dtheta_2; dtheta_3];

% accelerations:
syms ddx_top ddy_top ddtheta_1 ddtheta_2 ddtheta_3 real
ddq = [ddx_top; ddy_top; ddtheta_1; ddtheta_2; ddtheta_3];

% generalized force: we have two motors in the shoulder and hip
% CHECK HERE
syms torque_shoulder torque_hip real
% Q = [0; 0; 0; torque_shoulder; torque_hip]; 

% the input should be 2*1 vector [u1;u2]
u = sym('u',[2,1],'real');
% S is the selection matrix, Q = Su;S is 5*2 matrix u is 2*1 matrix; Q is
% 5*1 matrix
S = [0,0;0,0;0,0;1,0;0,1];

% position of all links
% x-coordinates of CoM:
syms x_com_top x_com_mid x_com_bot real

% y-coordinates of CoM:
syms y_com_top y_com_mid y_com_bot real

% Dimensions: CoM:
syms r_com_top r_com_mid r_com_bot real

% Dimensions: length of link:
syms l_top l_mid l_bot real

% x-coordinates of tip:
syms x_tip_top x_tip_mid x_tip_bot real

% y-coordinates of tip:
syms y_tip_top y_tip_mid y_tip_bot real

% Mass of top, mid, bot link: motor1 is shoulder; motor 2 is hip
syms m_top m_mid m_bot m_motor1 m_motor2 real

% Rotational inertia of top, mid, bot link:
syms I_top I_mid I_bot real

% Viscous damping at each joint:
b = sym('b',[numel(q),1],'real');

% Other variables
g = sym('g','real'); % gravity
t = sym('t','real'); % time

% pendulum location:

% compute the (x,y) location of the top link's CoM:
x_com_top = x_top + r_com_top*sin(theta_1);
y_com_top = y_top - r_com_top*cos(theta_1);

% compute the (x,y) location of the top link's tip:
x_tip_top = x_top + l_top*sin(theta_1);
y_tip_top = y_top - l_top*cos(theta_1);

%compute the (x,y) location of the mid link's CoM:
x_com_mid = x_tip_top + r_com_mid*sin(theta_1+theta_2);
y_com_mid = y_tip_top - r_com_mid*cos(theta_1+theta_2);

%compute the (x,y) location of the mid link's tip:
x_tip_mid = x_tip_top + l_mid*sin(theta_1+theta_2);
y_tip_mid = y_tip_top - l_mid*cos(theta_1+theta_2);

%compute the (x,y) location of the bot link's CoM:
x_com_bot = x_tip_mid + r_com_bot*sin(theta_1+theta_2+theta_3);
y_com_bot = y_tip_mid - r_com_bot*cos(theta_1+theta_2+theta_3);

%compute the (x,y) location of the bot link's tip:
x_tip_bot = x_tip_mid + l_bot*sin(theta_1+theta_2+theta_3);
y_tip_bot = y_tip_mid - l_bot*cos(theta_1+theta_2+theta_3);

% create a 2x6 array to hold all forward kinematics (FK) outputs:
% all position is store in a file
FK = [x_com_top, x_tip_top, x_com_mid, x_tip_mid, x_com_bot, x_tip_bot;
      y_com_top, y_tip_top, y_com_mid, y_tip_mid, y_com_bot, y_tip_bot];

% generate a MATLAB function to compute all the FK outputs:
matlabFunction(FK,'File','autogen_fwd_kin');




%% Derivative, Kinetic and Potential Energy
derivative = @(in)( jacobian(in,[q;dq])*[dq;ddq] );

% CoM x-velocities:
syms dx_com_top dx_com_mid dx_com_bot real
% CoM y-velocities:
syms dy_com_top dy_com_mid dy_com_bot real

dx_com_top = derivative(x_com_top); % x velocity top link
dy_com_top = derivative(y_com_top); % y velocity top link
dx_com_mid = derivative(x_com_mid); % x velocity mid link
dy_com_mid = derivative(y_com_mid); % y velocity mid link
dx_com_bot = derivative(x_com_bot); % x velocity bot link
dy_com_bot = derivative(y_com_bot); % y velocity bot link

% Kinetic energy 
syms ke_top ke_mid ke_bot KE real

% kinetic energy of each link:
ke_top = (1/2)*m_top*(dx_com_top^2 + dy_com_top^2) + (1/2)*I_top*(dtheta_1^2); 
ke_mid = (1/2)*(m_mid + m_motor1 + m_motor2)*(dx_com_mid^2 + dy_com_mid^2) + (1/2)*I_mid*((dtheta_1+dtheta_2)^2);
ke_bot = (1/2)*m_bot*(dx_com_bot^2 + dy_com_bot^2) + (1/2)*I_bot*((dtheta_1+dtheta_2+dtheta_3)^2);

KE = ke_top + ke_mid + ke_bot;

% Potential energy 
syms pe_top pe_mid pe_bot PE real

% potential energy of each link:
pe_top = m_top * g * y_com_top;
pe_mid = (m_mid + m_motor1 + m_motor2) * g * y_com_mid;
pe_bot = m_bot * g * y_com_bot;

%total potential energy:
PE = pe_top + pe_mid + pe_bot;

% Total Energy
syms Energy real
Energy =  PE + KE;

matlabFunction(KE,'File','autogen_KE');
matlabFunction(PE,'File','autogen_PE');
matlabFunction(Energy,'File','autogen_Energy');

%% Lagrangian Equation

% Lagrangian
syms L real
L = KE - PE;

% Euler-Lagrange equations
ELeq_term1  = sym('ELeq_term1',[numel(q),1],'real'); % d(del L/del dq)/dt
ELeq_term2  = sym('ELeq_term2',[numel(q),1],'real'); % del L/del qa
ELeq_damping= sym('ELeq_damping',[numel(q),1],'real'); % joint damping
ELeq_LHS    = sym('ELeq_LHS',[numel(q),1],'real');

for i = 1:numel(q)
    ELeq_term1(i) = simplify(derivative(diff(L,dq(i))));%diff is differences and approximate derivatives
    ELeq_term2(i) = simplify(diff(L,q(i))); % derivative is output time derivative of input
    ELeq_damping(i) = -b(i)*dq(i);
    ELeq_LHS(i) = ELeq_term1(i) - ELeq_term2(i) - ELeq_damping(i);
    ELeq_LHS(i) = simplify(ELeq_LHS(i));
end

% Put E-L eq in manipulator form
M = sym('M',[numel(q),numel(q)],'real');
C = sym('C',[numel(q),numel(q)],'real');
G = sym('G',[numel(q),1],'real');
for i = 1:numel(q)
    for j = 1:numel(q)
        M(i,j) = diff(ELeq_LHS(i),ddq(j)); 
        M(i,j) = simplify(M(i,j));
    end
    G(i) = diff(ELeq_LHS(i),g)*g;
    for j = 1:numel(q)
        C(i,j) = diff((ELeq_LHS(i) - (M(i,:)*ddq + G(i))),dq(j));
    end
    C(i) = simplify(C(i)); 
end

% compute and store inv(M)(inverse matrix)
% it's difficult to calculate, so I store Minv in savMinv.mat file;
% the function is save('saveMinv','Minv');
% Minv = sym('Minv',[numel(q),numel(q)],'real');
% Minv = simplify(inv(M));

% This function is used to load the Minv file;
matfile_Minv = matfile('saveMinv.mat');
Minv = matfile_Minv.Minv;
matlabFunction(M,'File','autogen_mass_matrix');
matlabFunction(C,'File','autogen_coriolis_matrix');
matlabFunction(G,'File','autogen_grav_vector');



%% Generate (nonlinear) state-space model from manipulator equation
f_ss = sym('f_ss',[2*numel(q),1],'real'); % drift vector field 
g_ss = sym('g_ss',[2*numel(q),2],'real'); % control vector field
% temp_drift = simplify(-Minv*(C*dq + G)); % it runs about 1 hour
matfile_drift = matfile('savetemp_drift.mat');
temp_drift = matfile_drift.temp_drift;
%temp_ctrl = simplify(Minv*S); 
matfile_ctrl = matfile('savetemp_ctrl.mat');
temp_ctrl = matfile_ctrl.temp_ctrl;

% Build state-space representation:
for i = 1:numel(q)
    f_ss(i) = dq(i);
    g_ss(i,:) = 0;
    f_ss(i+numel(q)) = temp_drift(i);
    g_ss(i+numel(q),:) = temp_ctrl(i,:);
end

matlabFunction(f_ss,'File','autogen_drift_vector_field');
matlabFunction(g_ss,'File','autogen_control_vector_field');

%% Linearize the state-space model around the upright equilibrium
% I do not know the equilibrium state of falling down! ?








