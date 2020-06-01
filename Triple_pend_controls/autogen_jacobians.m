function [dx_com_top,dy_com_top,dx_com_mid,dy_com_mid,dx_com_bot,dy_com_bot] = autogen_jacobians(dtheta_1,dtheta_2,dtheta_3,dx,dy,l_mid,l_top,r_com_mid,r_com_bot,r_com_top,theta_1,theta_2,theta_3)
%AUTOGEN_JACOBIANS
%    [DX_COM_TOP,DY_COM_TOP,DX_COM_MID,DY_COM_MID,DX_COM_BOT,DY_COM_BOT] = AUTOGEN_JACOBIANS(DTHETA_1,DTHETA_2,DTHETA_3,DX,DY,L_MID,L_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    31-May-2020 13:19:28

t2 = cos(theta_1);
dx_com_top = dx+dtheta_1.*r_com_top.*t2;
if nargout > 1
    t3 = sin(theta_1);
    dy_com_top = dy+dtheta_1.*r_com_top.*t3;
end
if nargout > 2
    t4 = theta_1+theta_2;
    t5 = l_top.*t2;
    t6 = cos(t4);
    dx_com_mid = dx+dtheta_1.*(t5+r_com_mid.*t6)+dtheta_2.*r_com_mid.*t6;
end
if nargout > 3
    t7 = l_top.*t3;
    t8 = sin(t4);
    dy_com_mid = dy+dtheta_1.*(t7+r_com_mid.*t8)+dtheta_2.*r_com_mid.*t8;
end
if nargout > 4
    t9 = t4+theta_3;
    t10 = cos(t9);
    t11 = sin(t9);
    t12 = l_mid.*t6;
    t13 = l_mid.*t8;
    t14 = r_com_bot.*t11;
    t15 = r_com_bot.*t10;
    dx_com_bot = dx+dtheta_2.*(t12+t15)+dtheta_3.*t15+dtheta_1.*(t5+t12+t15);
end
if nargout > 5
    dy_com_bot = dy+dtheta_2.*(t13+t14)+dtheta_3.*t14+dtheta_1.*(t7+t13+t14);
end
