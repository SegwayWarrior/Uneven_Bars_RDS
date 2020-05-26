%% plot_robot.m
%
% Description:
%   Plots the robot in its current configuration.
function plot_robot(q,params,varargin)
p = inputParser;

% Step 2: create the parsing schema:
addRequired(p,'triple_pendulum_release_config', ...
    @(q) isnumeric(q) && size(q,1)==3 && size(q,2)==1); % see if q is 3*1 vector
addRequired(p,'triple_pendulum_release_params', ...
    @(params) ~isempty(params)); % to see if params is empty 
%      2b: optional inputs:
addParameter(p, 'new_fig', false); % if true, plot will be on a new figure

% Step 3: parse the inputs:
parse(p, q, params, varargin{:});

% Verification: display the results of parsing:
% disp(p.Results)

%% Evaluate forward kinematics at points of interest
FK = fwd_kin(q,params);

% This is catching the bar: all the position is relative to the bar
% position

x_top = params.model.higher.top.x;
y_top = params.model.higher.top.y;


% (x,y) location of top link CoM:
top.curr.com.x = FK(1,1) + params.model.higher.top.x;
top.curr.com.y = FK(2,1) + params.model.higher.top.y;

% (x,y) location of top link tip:
top.curr.tip.x = FK(1,2)+ params.model.higher.top.x;
top.curr.tip.y = FK(2,2)+ params.model.higher.top.y;

% (x,y) location of mid link CoM:
mid.curr.com.x = FK(1,3) + params.model.higher.top.x;
mid.curr.com.y = FK(2,3) + params.model.higher.top.y;

% (x,y) location of mid link tip:
mid.curr.tip.x = FK(1,4) + params.model.higher.top.x;
mid.curr.tip.y = FK(2,4) + params.model.higher.top.y;

% (x,y) location of bot link CoM:
bot.curr.com.x = FK(1,5)+ params.model.higher.top.x;
bot.curr.com.y = FK(2,5) + params.model.higher.top.y;

% (x,y) location of bot link tip:
bot.curr.tip.x = FK(1,6) +params.model.higher.top.x;
bot.curr.tip.y = FK(2,6) + params.model.higher.top.y;

%% Display pendulum, and the pendulum's CoM
if p.Results.new_fig
    figure;
end

plot([x_top,top.curr.tip.x],[y_top,top.curr.tip.y],'Color',params.viz.colors.top,'LineWidth',8);
hold on;
plot([top.curr.tip.x,mid.curr.tip.x],[top.curr.tip.y,mid.curr.tip.y],'Color',params.viz.colors.mid,'LineWidth',8);
hold on;
plot([mid.curr.tip.x,bot.curr.tip.x],[mid.curr.tip.y,bot.curr.tip.y],'Color',params.viz.colors.bot,'LineWidth',8);
hold on;
scatter(top.curr.tip.x,top.curr.tip.y,params.model.geom.motor.mr1,...
    'MarkerEdgeColor',params.viz.colors.motor.edge1,...
    'MarkerFaceColor',params.viz.colors.motor.face1);
hold on;
scatter(mid.curr.tip.x,mid.curr.tip.y,params.model.geom.motor.mr2,...
    'MarkerEdgeColor',params.viz.colors.motor.edge2,...
    'MarkerFaceColor',params.viz.colors.motor.face2);
% scatter the CoM points
scatter(top.curr.com.x,top.curr.com.y,params.model.geom.motor.mr1/2,'x',...
    'MarkerEdgeColor',[1,1,1],...
    'MarkerFaceColor',[1,1,1]);
hold on;
scatter(mid.curr.com.x,mid.curr.com.y,params.model.geom.motor.mr2/2,'x',...
    'MarkerEdgeColor',[1,1,1],...
    'MarkerFaceColor',[1,1,1]);
hold on;
scatter(bot.curr.com.x,bot.curr.com.y,params.model.geom.motor.mr2/2,'x',...
    'MarkerEdgeColor',[1,1,1],...
    'MarkerFaceColor',[1,1,1]);
% hold off;

% add the lower and higher bar into

hold on;
scatter(params.model.lower.top.x,params.model.lower.top.y,params.model.geom.motor.mr2,...
    'MarkerEdgeColor',[0,0,0],...
    'MarkerFaceColor',[0,0,0]);
hold on;
scatter(params.model.higher.top.x,params.model.higher.top.y,params.model.geom.motor.mr2,...
    'MarkerEdgeColor',[0,0,0],...
    'MarkerFaceColor',[0,0,0]);
hold on;
plot([params.model.lower.top.x, params.model.lower.bot.x],[params.model.lower.top.y, params.model.lower.bot.y],'Color',[0,0,0],'LineWidth',2);
hold on;
plot([params.model.higher.top.x, params.model.higher.bot.x],[params.model.higher.top.y, params.model.higher.bot.y],'Color',[0,0,0],'LineWidth',2);
hold off;
axis(params.viz.axis_lims);
daspect([1 1 1]) % no distortion

xlabel('$x$');
ylabel('$y$');























end