%% Event function for ODE45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
%   Event function that is called when a constraint becomes active or inactive
%
% Inputs:
%   t and x are required, but not used
%   events is shared with parent function
%
% Outputs:
%   value
%   isterminal
%   direction
function [value,isterminal,direction] = robot_events(~,~)

   value = events;
   isterminal = ones(4,1);
   direction = -ones(4,1);

end