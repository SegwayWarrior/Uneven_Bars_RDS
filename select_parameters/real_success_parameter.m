para = matfile("parameter_matrix");
para = para.parameter_matrix;
success_param = [];
loop_num = 0;
for i = 1: size(para,1)
    if para(i,11) == 1
        loop_num = loop_num + 1
        success_param(loop_num,:) = para(i,:);
    end
end

save("success_param","success_param");

init_env();
catch_range = 0.1;
params = init_params_success_parameter;

x_top = 0;
y_top = 0;
dx_top = 0;
dy_top = 0;

loop_num = 0 ; 
parameter_matrix = [];

tspan_passive = 0:params.sim.dt:1.5;

test_number = 0;
for ii = 1: size(success_param,1)
    test_number = test_number + 1
    x_IC = success_param(ii,1:10)';
    [tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics_release(...
                                                    t,x,0,params,'controller','passive'),...
                                                    tspan_passive, x_IC');
    hold on;
    plot(xsim_passive(:,1),xsim_passive(:,2),'o-',...
                    'Color',params.viz.colors.tracers.bot,...
                    'MarkerSize',0.05,'LineWidth',0.05,...
                    'MarkerFaceColor',params.viz.colors.tracers.bot,...
                    'MarkerEdgeColor',params.viz.colors.tracers.bot);
                     hold on;

end






