%% total_energy.m
%
% Description:
%   Wrapper function for autogen_potential_energy.m
%   Computes the potential energy of the Uneven Bars robot

function TE = total_energy(x,params)

y = x(2);
theta_1 = x(3);
theta_2 = x(4);
theta_3 = x(5);
dx = x(6);
dy = x(7);
dtheta_1 = x(8);
dtheta_2 = x(9);
dtheta_3 = x(10);

TE = autogen_total_energy(params.model.dyn.bot.I,...
                          params.model.dyn.mid.I,...
                          params.model.dyn.top.I,...
                          dtheta_1,...
                          dtheta_2,...
                          dtheta_3,...
                          dx,...
                          dy,...
                          params.model.dyn.g,...
                          params.model.geom.mid.l,...
                          params.model.geom.top.l,...
                          params.model.dyn.bot.m,...
                          params.model.dyn.mid.m,...
                          params.model.dyn.motor1.m,...
                          params.model.dyn.motor2.m,...
                          params.model.dyn.top.m,...
                          params.model.dyn.mid.r_com,...
                          params.model.dyn.bot.r_com,...
                          params.model.dyn.top.r_com,...
                          theta_1,...
                          theta_2,...
                          theta_3,...
                          y);

end