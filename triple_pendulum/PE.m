%% PE.m
% Inputs:
%   x: the state vector, x = [theta1;theta2;theta3;dtheta1;dtheta2;dtheta3];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   pe: potential energy
function pe = PE(x,params)
theta_1 = x(1);
theta_2 = x(2);
theta_3 = x(3);

pe = autogen_PE( params.model.dyn.g,...
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
                                    theta_3);

end