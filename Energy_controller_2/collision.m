%% collision

% inputs: 1x1 t, 1x10 x, params
% outputs: dq at t+ (time after collision)

function dq_plus = collision(t, x, params)

nq = numel(x)/2;
q = x(1:nq);
q_dot = x(nq+1: 2*nq);

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
H = H_eom(x, params);

% 5x5 M matrix
M = mass_matrix_uneven(x, params);

% 5x5 inverse M matrix
Minv = inv(M);

[A_all, Hessian] = constraint_derivatives(x, params);

% A matrix
A = A_all([1,2],:);

% coefficient of restitution
eps = params.sim.coef.restit;

delta_dq = -Minv*A'*inv(A*Minv*A')*(1+eps)*A*q_dot;
dq_plus = delta_dq + q_dot;
end
