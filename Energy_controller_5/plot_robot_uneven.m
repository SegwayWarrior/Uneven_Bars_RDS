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

xx = q(1);
yy = q(2);
theta_1 = q(3);
theta_2 = q(4);
theta_3 = q(5);

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

% %plots the top link
% plot([xx top.curr.tip.x], [yy top.curr.tip.y],...
%     'LineWidth', 5, 'Color', params.viz.colors.top)
% hold on;
% 
% %plots the mid link
% plot([top.curr.tip.x mid.curr.tip.x], [top.curr.tip.y mid.curr.tip.y],...
%     'LineWidth', 5, 'Color', params.viz.colors.mid)
% hold on;
% 
% %plots the bot link
% plot([mid.curr.tip.x bot.curr.tip.x], [mid.curr.tip.y bot.curr.tip.y],...
%     'LineWidth', 5, 'Color', params.viz.colors.bot)
% hold on;
% 
% %plots the top CoM
% scatter(top.curr.com.x, top.curr.com.y, 100,...
%     'MarkerFaceColor', params.viz.colors.top,...
%     'MarkerEdgeColor', [0 0 0]);
% hold on;

% %plots the mid CoM
% scatter(mid.curr.com.x, mid.curr.com.y, 100,...
%     'MarkerFaceColor', params.viz.colors.mid,...
%     'MarkerEdgeColor', [0 0 0]);
% hold on;

% %plots the bot CoM
% scatter(bot.curr.com.x, bot.curr.com.y, 100,...
%     'MarkerFaceColor',params.viz.colors.bot,...
%     'MarkerEdgeColor',[0 0 0]);
% hold off;

%plots the bars
scatter(0, 0, 100,...
    'MarkerFaceColor', params.viz.colors.mid,...
    'MarkerEdgeColor', [0 0 0]);
hold on;
scatter(2.9, 1.2, 100,...
    'MarkerFaceColor', params.viz.colors.mid,...
    'MarkerEdgeColor', [0 0 0]);
hold on;

% load images
[top_img, top_map, top_alphachannel] = imread('image74.png');
[mid_img, mid_map, mid_alphachannel] = imread('image73.png');
[bot_img, bot_map, bot_alphachannel] = imread('image72.png');

% define angles
theta_top = rad2deg(theta_1);
theta_mid = rad2deg(theta_2 + theta_1);
theta_bot = rad2deg(theta_1+ theta_2 + theta_3);

%% Plot Link 1

if (rem(theta_top,360)>0) && (rem(theta_top,360)<90)|| ...
   (rem(theta_top,360)>-360) && (rem(theta_top,360)<-270)
    
    % for when theta_top is is in the 1st quadrant
    
    top_img = imrotate(top_img,-theta_top+180);
    top_alphachannel = imrotate(top_alphachannel,-theta_top+180);

    top_corner1 = [xx - (2/7)*cosd(theta_top), top.curr.tip.y - (2/7)*sind(theta_top)];
    top_corner2 = [top.curr.tip.x + (2/7)*cosd(theta_top), yy + (2/7)*sind(theta_top)];

    image([top_corner1(1) top_corner2(1)],[top_corner1(2) top_corner2(2)],top_img,'AlphaData', top_alphachannel)
    
elseif (rem(theta_top,360)>90) && (rem(theta_top,360)<180)|| ...
       (rem(theta_top,360)>-270) && (rem(theta_top,360)<-180)
    
    % for when theta_top is is in the 2nd quadrant
    
    top_img = imrotate(top_img,theta_top);
    top_alphachannel = imrotate(top_alphachannel,theta_top);

    top_corner1 = [xx + (2/7)*cosd(theta_top), top.curr.tip.y + (2/7)*sind(theta_top)];
    top_corner2 = [top.curr.tip.x - (2/7)*cosd(theta_top), yy - (2/7)*sind(theta_top)];

    image([top_corner1(1) top_corner2(1)],[top_corner1(2) top_corner2(2)],top_img,'AlphaData', top_alphachannel)

elseif (rem(theta_top,360)>180) && (rem(theta_top,360)<270)|| ...
       (rem(theta_top,360)>-180) && (rem(theta_top,360)<-90)
    
    % for when theta_top is in the 3rd quadrant
    
    top_img = imrotate(top_img,-theta_top);
    top_alphachannel = imrotate(top_alphachannel,-theta_top);

    top_corner1 = [xx - (2/7)*cosd(theta_top), top.curr.tip.y - (2/7)*sind(theta_top)];
    top_corner2 = [top.curr.tip.x + (2/7)*cosd(theta_top), yy + (2/7)*sind(theta_top)];

    image([top_corner1(1) top_corner2(1)],[top_corner1(2) top_corner2(2)],top_img,'AlphaData', top_alphachannel)
   
elseif (rem(theta_top,360)>270) && (rem(theta_top,360)<360)|| ...
       (rem(theta_top,360)>-90) && (rem(theta_top,360)<0)
    
    % for when theta_top is is in the 4th quadrant
    
    top_img = imrotate(top_img,theta_top+180);
    top_alphachannel = imrotate(top_alphachannel,theta_top+180);

    top_corner1 = [xx + (2/7)*cosd(theta_top), top.curr.tip.y + (2/7)*sind(theta_top)];
    top_corner2 = [top.curr.tip.x - (2/7)*cosd(theta_top), yy - (2/7)*sind(theta_top)];

    image([top_corner1(1) top_corner2(1)],[top_corner1(2) top_corner2(2)],top_img,'AlphaData', top_alphachannel)
    
end

%% Plotting Link 2

if (rem(theta_mid,360)>0) && (rem(theta_mid,360)<90)|| ...
   (rem(theta_mid,360)>-360) && (rem(theta_mid,360)<-270)
    
    % for when theta_mid is is in the 1st quadrant
    
    mid_img = imrotate(mid_img,-theta_mid+180);
    mid_alphachannel = imrotate(mid_alphachannel,-theta_mid+180);

    mid_corner1 = [top.curr.tip.x - (2/7)*cosd(theta_mid), mid.curr.tip.y - (2/7)*sind(theta_mid)];
    mid_corner2 = [mid.curr.tip.x + (2/7)*cosd(theta_mid), top.curr.tip.y + (2/7)*sind(theta_mid)];

    image([mid_corner1(1) mid_corner2(1)],[mid_corner1(2) mid_corner2(2)],mid_img,'AlphaData', mid_alphachannel)
    
elseif (rem(theta_mid,360)>90) && (rem(theta_mid,360)<180)|| ...
       (rem(theta_mid,360)>-270) && (rem(theta_mid,360)<-180)
    
    % for when theta_mid is is in the 2nd quadrant
    
    mid_img = imrotate(mid_img,theta_mid);
    mid_alphachannel = imrotate(mid_alphachannel,theta_mid);

    mid_corner1 = [top.curr.tip.x + (2/7)*cosd(theta_mid), mid.curr.tip.y + (2/7)*sind(theta_mid)];
    mid_corner2 = [mid.curr.tip.x - (2/7)*cosd(theta_mid), top.curr.tip.y - (2/7)*sind(theta_mid)];

    image([mid_corner1(1) mid_corner2(1)],[mid_corner1(2) mid_corner2(2)],mid_img,'AlphaData', mid_alphachannel)

elseif (rem(theta_mid,360)>180) && (rem(theta_mid,360)<270)|| ...
       (rem(theta_mid,360)>-180) && (rem(theta_mid,360)<-90)
    
    % for when theta_mid is is in the 3rd quadrant
    
    mid_img = imrotate(mid_img,-theta_mid);
    mid_alphachannel = imrotate(mid_alphachannel,-theta_mid);

    mid_corner1 = [top.curr.tip.x - (2/7)*cosd(theta_mid), mid.curr.tip.y - (2/7)*sind(theta_mid)];
    mid_corner2 = [mid.curr.tip.x + (2/7)*cosd(theta_mid), top.curr.tip.y + (2/7)*sind(theta_mid)];

    image([mid_corner1(1) mid_corner2(1)],[mid_corner1(2) mid_corner2(2)],mid_img,'AlphaData', mid_alphachannel)
   
elseif (rem(theta_mid,360)>270) && (rem(theta_mid,360)<360)|| ...
       (rem(theta_mid,360)>-90) && (rem(theta_mid,360)<0)
    
    % for when theta_mid is is in the 4th quadrant
    
    mid_img = imrotate(mid_img,theta_mid+180);
    mid_alphachannel = imrotate(mid_alphachannel,theta_mid+180);

    mid_corner1 = [top.curr.tip.x + (2/7)*cosd(theta_mid), mid.curr.tip.y + (2/7)*sind(theta_mid)];
    mid_corner2 = [mid.curr.tip.x - (2/7)*cosd(theta_mid), top.curr.tip.y - (2/7)*sind(theta_mid)];

    image([mid_corner1(1) mid_corner2(1)],[mid_corner1(2) mid_corner2(2)],mid_img,'AlphaData', mid_alphachannel)
    
end

%% Plot Link 3

if (rem(theta_bot,360)>0) && (rem(theta_bot,360)<90)|| ...
   (rem(theta_bot,360)>-360) && (rem(theta_bot,360)<-270)
    
    % for when theta_bot is is in the 1st quadrant
    
    bot_img = imrotate(bot_img,-theta_bot+180);
    bot_alphachannel = imrotate(bot_alphachannel,-theta_bot+180);

    bot_corner1 = [mid.curr.tip.x - (2/7)*cosd(theta_bot), bot.curr.tip.y - (2/7)*sind(theta_bot)];
    bot_corner2 = [bot.curr.tip.x + (2/7)*cosd(theta_bot), mid.curr.tip.y + (2/7)*sind(theta_bot)];

    image([bot_corner1(1) bot_corner2(1)],[bot_corner1(2) bot_corner2(2)],bot_img,'AlphaData', bot_alphachannel)
    
elseif (rem(theta_bot,360)>90) && (rem(theta_bot,360)<180)|| ...
       (rem(theta_bot,360)>-270) && (rem(theta_bot,360)<-180)
    
    % for when theta_mid is is in the 2nd quadrant
    
    bot_img = imrotate(bot_img,theta_bot);
    bot_alphachannel = imrotate(bot_alphachannel,theta_bot);

    bot_corner1 = [mid.curr.tip.x + (2/7)*cosd(theta_bot), bot.curr.tip.y + (2/7)*sind(theta_bot)];
    bot_corner2 = [bot.curr.tip.x - (2/7)*cosd(theta_bot), mid.curr.tip.y - (2/7)*sind(theta_bot)];

    image([bot_corner1(1) bot_corner2(1)],[bot_corner1(2) bot_corner2(2)],bot_img,'AlphaData', bot_alphachannel)

elseif (rem(theta_bot,360)>180) && (rem(theta_bot,360)<270)|| ...
       (rem(theta_bot,360)>-180) && (rem(theta_bot,360)<-90)
    
    % for when theta_mid is is in the 3rd quadrant
    
    bot_img = imrotate(bot_img,-theta_bot);
    bot_alphachannel = imrotate(bot_alphachannel,-theta_bot);

    bot_corner1 = [mid.curr.tip.x - (2/7)*cosd(theta_bot), bot.curr.tip.y - (2/7)*sind(theta_bot)];
    bot_corner2 = [bot.curr.tip.x + (2/7)*cosd(theta_bot), mid.curr.tip.y + (2/7)*sind(theta_bot)];

    image([bot_corner1(1) bot_corner2(1)],[bot_corner1(2) bot_corner2(2)],bot_img,'AlphaData', bot_alphachannel)
   
elseif (rem(theta_bot,360)>270) && (rem(theta_bot,360)<360)|| ...
       (rem(theta_bot,360)>-90) && (rem(theta_bot,360)<0)
    
    % for when theta_bot is is in the 4th quadrant
    
    bot_img = imrotate(bot_img,theta_bot+180);
    bot_alphachannel = imrotate(bot_alphachannel,theta_bot+180);

    bot_corner1 = [mid.curr.tip.x + (2/7)*cosd(theta_bot), bot.curr.tip.y + (2/7)*sind(theta_bot)];
    bot_corner2 = [bot.curr.tip.x - (2/7)*cosd(theta_bot), mid.curr.tip.y - (2/7)*sind(theta_bot)];

    image([bot_corner1(1) bot_corner2(1)],[bot_corner1(2) bot_corner2(2)],bot_img,'AlphaData', bot_alphachannel)
    
end

hold off;

axis(params.viz.axis_lims);
daspect([1 1 1]) % no distortion

xlabel('$x$');
ylabel('$y$');

end