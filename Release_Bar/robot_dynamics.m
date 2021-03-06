%% robot_dynamics.m
% Description:
%   Computes the right-hand side of the state-space dynamics
%       dx = f_ss(x) + g_ss(x)*u
% Inputs:
%   t: time (scalar)
%   x: the state vector, x = [theta1;theta2;theta3;dtheta1;dtheta2;dtheta3];
%   u_ff: feedforward control input (scalar, horizontal force applied to
%       the cart)
%   params: a struct with many elements, generated by calling init_params.m
%   varargin: optional name-value pair arguments:
%       'controller': (default: 'passive') can have three values:
%           'passive': no feedback control, but calling function can still
%               supply a feedforward control u_ff;
%           'stabilize': a feedback controller is used to stabilize the
%               inverted equilibrium,
%               gain matrix stored in params.control.inverted.K;
%           'swingup': an energy-shaping feedback controller is used to
%               bring the system to a homoclinic orbit,
%               gain matrix stored in params.control.swingup.K;


function [dx] = robot_dynamics(t,x,u_ff,params,varargin)
% Step 1: instantiate an inputParser:
p = inputParser;
% Step 2: create the parsing schema:
addRequired(p,'triple_pendulum_time',...
    @(t) isnumeric(t) && size(t,1)==1);
addRequired(p,'triple_pendulum_state', ...
    @(x) isnumeric(x) && size(x,1)==10 && size(x,2)==1);
addRequired(p,'triple_pendulum_u_ff',...
    @(u_ff) isnumeric(u_ff) && size(u_ff,1)==1 && size(u_ff,2)==1);
addRequired(p,'triple_pendulum_params', ...
    @(params) ~isempty(params));
addParameter(p, 'controller', 'passive');
parse(p, t,x,u_ff,params,varargin{:});

% Finally, actually compute the controls + dynamics:
    switch p.Results.controller
        case 'passive'
            u_fb = 0;
        case 'stabilize'
            x_ref = [0;0;0;0;0;0;0;0;0;0]; % I do not know the stabilize stage
            u_fb = -params.control.inverted.K*(x - x_ref);
        case 'swingup'
            u_fb = 0;
        otherwise
            u_fb = 0;
    end

    u = u_ff + u_fb;

    f_ss = drift_vector_field(x,params);
    g_ss = control_vector_field(x,params);
    dx = f_ss + g_ss*[0;0];

end





