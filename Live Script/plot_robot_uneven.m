%% plot_robot_uneven.m
%
% Description:
%   Plots the robot in its current configuration.
%   
% Inputs:
%   q: robot configuration, q = [x; y; theta_1; theta_2; theta_3];
%
% Outputs:
%   none
%

function plot_robot_uneven(q,params,varargin)
%% Parse input arguments
% Note: a simple robot plotting function doesn't need this, but I want to
% write extensible code, so I'm using "varargin" which requires input
% parsing. See the reference below:
%
% https://people.umass.edu/whopper/posts/better-matlab-functions-with-the-inputparser-class/

% Step 1: instantiate an inputParser:
p = inputParser;

% Step 2: create the parsing schema:
%      2a: required inputs:
addRequired(p,'robot_config', ...
    @(q) isnumeric(q) && size(q,1)==10 && size(q,2)==1);
addRequired(p,'robot_params', ...
    @(params) ~isempty(params));
%      2b: optional inputs:
addParameter(p, 'new_fig', false); % if true, plot will be on a new figure

% Step 3: parse the inputs:
parse(p, q, params, varargin{:});

% Verification: display the results of parsing:
% disp(p.Results)

%% for convenience, define each generalized coordinate

%% Evaluate forward kinematics at points of interest
FK = fwd_kin_uneven(q,params);

% (x,y) location of top link CoM:
top.curr.com.x = FK(1,1);
top.curr.com.y = FK(2,1);

% (x,y) location of top link tip:
top.curr.tip.x = FK(1,2);
top.curr.tip.y = FK(2,2);

% (x,y) location of mid link CoM:
mid.curr.com.x = FK(1,3);
mid.curr.com.y = FK(2,3);

% (x,y) location of mid link tip:
mid.curr.tip.x = FK(1,4);
mid.curr.tip.y = FK(2,4);

% (x,y) location of bot link CoM:
bot.curr.com.x = FK(1,5);
bot.curr.com.y = FK(2,5);

% (x,y) location of bot link tip:
bot.curr.tip.x = FK(1,6);
bot.curr.tip.y = FK(2,6);

%% Display the top, middle and bottom links
if p.Results.new_fig
    figure;
end

%plots the top link
plot([0 top.curr.tip.x], [0 top.curr.tip.y],...
    'LineWidth', 5, 'Color', params.viz.colors.top)
hold on;

%plots the mid link
plot([top.curr.tip.x mid.curr.tip.x], [top.curr.tip.y mid.curr.tip.y],...
    'LineWidth', 5, 'Color', params.viz.colors.mid)
hold on;

%plots the bot link
plot([mid.curr.tip.x bot.curr.tip.x], [mid.curr.tip.y bot.curr.tip.y],...
    'LineWidth', 5, 'Color', params.viz.colors.bot)
hold on;

%plots the top CoM
scatter(top.curr.com.x, top.curr.com.y, 100,...
    'MarkerFaceColor', params.viz.colors.top,...
    'MarkerEdgeColor', [0 0 0]);
hold on;

%plots the mid CoM
scatter(mid.curr.com.x, mid.curr.com.y, 100,...
    'MarkerFaceColor', params.viz.colors.mid,...
    'MarkerEdgeColor', [0 0 0]);
hold on;

%plots the bot CoM
scatter(bot.curr.com.x, bot.curr.com.y, 100,...
    'MarkerFaceColor',params.viz.colors.bot,...
    'MarkerEdgeColor',[0 0 0]);
hold off;

axis(params.viz.axis_lims);
daspect([1 1 1]) % no distortion

xlabel('$x$');
ylabel('$y$');

end