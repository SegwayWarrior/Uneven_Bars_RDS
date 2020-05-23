%% constraint_forces_uneven

% outputs: Fseg = a 2xlength(tseg) array of constraint forces

function [Fseg] = constraint_forces_uneven(tseg, xseg, params)

Fseg = zeros(2,length(tseg));

for i=1:length(tseg)
    t = tseg(i);
    x = xseg(:,i);
    q_dot = x(6:10);
    
    nq = numel(x)/2; % assuming x = [q, dq]
    q = x(1:nq);
    q_dot = x(nq+1: 2*nq);
    
    % solve for control inputs at this instant
    % no controls at this instant
    Q = [0; 0; 0; 0; 0];
    
    % finding parts that don't depend on constraint forces
    H = H_eom(x, params);
    M = mass_matrix_uneven(x, params);
    Minv = inv(M);
    [A_all, Hessian] = constraint_derivatives(x, params);
    
    % TRYING TO RELEASE ROBOT FROM BAR
    xx = q(1);
    yy = q(2);
    th1 = q(3);
    th2 = q(4);
    th3 = q(5);
    
    l1 = params.model.geom.top.l;
    l2 = params.model.geom.mid.l;
    l3 = params.model.geom.bot.l;
    
    p1_x = xx + l1*sin(th1);
    p1_y = yy - l1*cos(th1);
    
    p2_x = p1_x + l2*sin(th1+th2);
    p2_y = p1_y - l2*cos(th1+th2);
    
    p3_x = p2_x + l3*sin(th1+th2+th3);
    p3_y = p2_y - l3*sin(th1+th2+th3);
    
    if (p1_x > 0) && (p1_y < 0) && (p2_x > 0) && (p2_y < 0) && (p3_x > 0) && (p3_y < 0)
        Fseg(:,i) = zeros(2,0);
    else
        A = A_all([1,2],:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                    q_dot'*Hessian(:,:,2)*q_dot];
        F = inv(A*Minv*A')*(A*Minv*(Q - H) + Adotqdot);
        Fseg(:,i) = [F(1); F(2)];
    end
    
    % build the constraints and forces
%     switch params.sim.constraints 
%         % the robot is off the bar
%         case ['false', 'false']
%             Fseg(:,i) = zeros(2,0);
%             
%         % the robot is on the bar
%         case ['true', 'true',] 
%             A = A_all([1,2],:);
%             Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
%                         q_dot'*Hessian(:,:,2)*q_dot];
%             F = inv(A*Minv*A')*(A*Minv*(Q - H) + Adotqdot);
%             Fseg(:,i) = [F(1); F(2)];
%     end

end
end