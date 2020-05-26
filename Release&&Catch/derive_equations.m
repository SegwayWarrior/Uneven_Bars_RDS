%% derive_equations.m
% triple_pendulum model and when catch another bar model
% function derive_equations
clear;
close all;
clc;
%% Define parameters
% theta_1: angle of top link, ccw+ wrt x-axis
% theta_2: angle of mid link, ccw+ wrt top link
% theta_3: angle of bot link, ccw+ wrt mid link
syms theta_1 theta_2 theta_3 real
q = [theta_1; theta_2; theta_3];

% velocities:
syms dtheta_1 dtheta_2 dtheta_3 real
dq = [dtheta_1; dtheta_2; dtheta_3];

% Generalized accelerations:
syms ddtheta_1 ddtheta_2 ddtheta_3 real
ddq = [ddtheta_1; ddtheta_2; ddtheta_3];

% the input should be 2*1 vector [u1;u2]
u = sym('u',[2,1],'real');

% S is the selection matrix, Q = Su;S is 3*2 matrix u is 2*1 matrix; Q is
% 3*1 matrix
S = [0,0;1,0;0,1];

% position of all links
% x-coordinates of the top, mid, bot links CoM:
syms x_com_top x_com_mid x_com_bot real

% y-coordinates of the top, mid, bot links CoM:
syms y_com_top y_com_mid y_com_bot real

% Dimensions: distance joint to top, mid, bot link CoM:
syms r_com_top r_com_mid r_com_bot real

% Dimensions: length of the top, mid, bot link
syms l_top l_mid l_bot real

% x-coordinates of top, mid, bot tip:
syms x_tip_top x_tip_mid x_tip_bot real

% y-coordinates of top, mid, bot tip:
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
x_com_top = r_com_top*sin(theta_1);
y_com_top = -r_com_top*cos(theta_1);

% compute the (x,y) location of the top link's tip:
x_tip_top = l_top*sin(theta_1);
y_tip_top = -l_top*cos(theta_1);

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
FK = [x_com_top, x_tip_top, x_com_mid, x_tip_mid, x_com_bot, x_tip_bot;
      y_com_top, y_tip_top, y_com_mid, y_tip_mid, y_com_bot, y_tip_bot];

% generate a MATLAB function to compute all the FK outputs:
matlabFunction(FK,'File','autogen_fwd_kin');


%% Derivative, Kinetic and Potential Energy
% from https://github.com/MatthewPeterKelly/OptimTraj/blob/master/demo/fiveLinkBiped/Derive_Equations.m
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

% total kinetic energy:
KE = ke_top + ke_mid + ke_bot;

% Potential energy
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

syms L real
L = KE - PE;
% Variable initializations:
ELeq_term1  = sym('ELeq_term1',[numel(q),1],'real'); % d(del L/del dq)/dt
ELeq_term2  = sym('ELeq_term2',[numel(q),1],'real'); % del L/del qa
ELeq_damping= sym('ELeq_damping',[numel(q),1],'real'); % joint damping
ELeq_LHS    = sym('ELeq_LHS',[numel(q),1],'real');

for i = 1:numel(q)
    ELeq_term1(i) = simplify(derivative(diff(L,dq(i))));
    ELeq_term2(i) = simplify(diff(L,q(i)));
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

% compute and store inv(M):
Minv = sym('Minv',[numel(q),numel(q)],'real');
Minv = simplify(inv(M));

matlabFunction(M,'File','autogen_mass_matrix');
matlabFunction(C,'File','autogen_coriolis_matrix');
matlabFunction(G,'File','autogen_grav_vector');

%% Generate (nonlinear) state-space model from manipulator equation

f_ss = sym('f_ss',[2*numel(q),1],'real'); % drift vector field
g_ss = sym('g_ss',[2*numel(q),2],'real'); % control vector field

temp_drift = simplify(-Minv*(C*dq + G)); % TODO, also use simplify(...)
temp_ctrl = simplify(Minv*S); % actuators at shoulders and hips

% Build state-space representation:
for i = 1:numel(q)
    f_ss(i) = dq(i);
    g_ss(i,:) = 0;
    f_ss(i+numel(q)) = temp_drift(i);
    g_ss(i+numel(q),:) = temp_ctrl(i,:);
end

matlabFunction(f_ss,'File','autogen_drift_vector_field');
matlabFunction(g_ss,'File','autogen_control_vector_field');

% the equalibrium may have some wrong in Unevenbar_RDS: theta_1 is
% theta_1_eq??




























