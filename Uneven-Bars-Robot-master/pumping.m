%% pumping.m 

function [x, Q] = pumping(x, M, C, G, params)

% defining variables
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
% kinEner = kinetic_energy(x,params);
% potEner = potential_energy(x,params);
totEner = total_energy(x,params);
potEner = potential_energy(x,params)

% desEner = 294.3426;  % max potential energy
desEner = 800; % gets robot to about 10 rad/sec

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

torq_mat = (M_aa-M_au*inv(M_uu)*M_ua)*qa_ddot - M_au*inv(M_uu)*(C_uu*qu_dot+C_ua*qa_dot+G_u)+C_au*qu_dot + C_aa*qa_dot + G_a ;

tau_shoulders = torq_mat(1);
tau_hips = torq_mat(2);

% set torque limits
torq_lim = 1.2;

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

% push back if past 90deg
if (th2 > pi/2) && (th2_dot >0)
    x(9) = -0.3*x(9);
elseif (th2 < -pi/2) && (th2_dot <0)
    x(9) = -0.3*x(9);
end
if (th3 > pi/2) && (th3_dot >0)
    x(10) = -0.3*x(10);
elseif (th3 < -pi/2) && (th3_dot <0)
    x(10) = -0.3*x(10);
end

th1pre = th1;

% 5x1 Q column vector
Q = [0;
     0; 
     0; 
     tau_shoulders; 
     tau_hips];
 
delta_t = params.sim.dt;
end