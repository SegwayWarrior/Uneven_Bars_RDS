function [A_all,H_x,H_y] = autogen_constraint_derivatives
%AUTOGEN_CONSTRAINT_DERIVATIVES
%    [A_ALL,H_X,H_Y] = AUTOGEN_CONSTRAINT_DERIVATIVES

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    31-May-2020 13:19:42

A_all = reshape([1.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0],[2,5]);
if nargout > 1
    H_x = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
if nargout > 2
    H_y = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
