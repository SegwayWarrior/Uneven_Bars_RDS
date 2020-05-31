%% Success_parmater.m

% this function is used to test different combination of initial parameter
% which success jump to another;
% There are 6 paramters(initially stored in init_paras_release.m);

% params.sim.ICs.theta_1 is theta1: theta1 is difficult to measure: so we
% just test theta1 = [pi/6; pi/4; pi/3; pi/2; pi*(2/3)];

% params.sim.ICs.dtheta_1  is the veloctiy of theta_1; 
% dtheta_1 = [0:0.5:3];

% params.sim.ICs.theta_2 = [-pi/2:pi/3:pi/2];
% params.sim.ICs.theta_3 = [-pi/2:pi/3:pi/2];
% params.sim.ICs.dtheta_2 = [-1:1:2];
% params.sim.ICs.dtheta_3 = [-1:1:2];

init_env();
catch_range = 0.2;
params = init_params_success_parameter;

x_top = 0;
y_top = 0;
dx_top = 0;
dy_top = 0;

loop_num = 0 ; 
parameter_matrix = [];

tspan_passive = 0:params.sim.dt:1.5;

for theta_1 = pi/3:pi/6:pi*(2/3)

for dtheta_1 = 0:0.5:3
    for theta_2 = -pi/2:pi/6:pi/2
        for theta_3 = -pi/2:pi/6:pi/2
            for dtheta_2 = -1:0.5:2
                for dtheta_3 = -1:0.5:2
                    success = 0;
                    loop_num = loop_num + 1
                    x_IC = [x_top;
                            y_top;
                            theta_1;
                            theta_2;
                            theta_3;
                            dx_top;
                            dy_top;
                            dtheta_1;
                            dtheta_2;
                            dtheta_3];
                     
                     [tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics_release(...
                                                    t,x,0,params,'controller','passive'),...
                                                    tspan_passive, x_IC');
                     catch_time = size(xsim_passive,1);
                        for ii = 1 : size(xsim_passive,1)
                            if ((xsim_passive(ii,1)- params.model.higher.top.x)^2 + (xsim_passive(ii,2)- params.model.higher.top.y)^2) < catch_range
                                catch_time = ii;
                                break;
                            end
                        end
                        if (catch_time < size(xsim_passive,1))
                            success = 1;
                        end
                     hold on;
                     if loop_num ==1
                     plot_robot_release(x_IC(1:5),params,'new_fig',false);
                     end
                     hold on;
                     plot(xsim_passive(:,1),xsim_passive(:,2),'o-',...
                    'Color',params.viz.colors.tracers.bot,...
                    'MarkerSize',0.01,'LineWidth',0.01,...
                    'MarkerFaceColor',params.viz.colors.tracers.bot,...
                    'MarkerEdgeColor',params.viz.colors.tracers.bot);
                     hold on;
                     
                     parameter_matrix(loop_num,:)=[x_top;
                            y_top;
                            theta_1;
                            theta_2;
                            theta_3;
                            dx_top;
                            dy_top;
                            dtheta_1;
                            dtheta_2;
                            dtheta_3;
                            success];
                 
                end
            end
        end
    end
end

end

save("parameter_matrix","parameter_matrix")

