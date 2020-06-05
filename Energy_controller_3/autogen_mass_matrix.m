function M = autogen_mass_matrix(I_bot,I_mid,I_top,l_mid,l_top,m_bot,m_mid,m_motor1,m_motor2,m_top,r_com_mid,r_com_bot,r_com_top,theta_1,theta_2,theta_3)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(I_BOT,I_MID,I_TOP,L_MID,L_TOP,M_BOT,M_MID,M_MOTOR1,M_MOTOR2,M_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    25-May-2020 09:36:58

t2 = cos(theta_1);
t3 = cos(theta_2);
t4 = cos(theta_3);
t5 = sin(theta_1);
t6 = theta_1+theta_2;
t7 = theta_2+theta_3;
t8 = l_mid.^2;
t9 = l_top.^2;
t10 = r_com_mid.^2;
t11 = r_com_bot.^2;
t26 = m_mid./2.0;
t27 = m_motor1./2.0;
t28 = m_motor2./2.0;
t32 = m_bot+m_mid+m_motor1+m_motor2+m_top;
t12 = l_mid.*t4;
t13 = cos(t6);
t14 = cos(t7);
t15 = sin(t6);
t16 = t6+theta_3;
t17 = m_top.*r_com_top.*t2;
t19 = m_top.*r_com_top.*t5;
t21 = m_bot.*t8;
t22 = m_bot.*t11;
t23 = m_mid.*t10;
t24 = m_motor1.*t10;
t25 = m_motor2.*t10;
t31 = l_mid.*l_top.*m_bot.*t3;
t33 = l_top.*m_mid.*r_com_mid.*t3;
t34 = l_top.*m_motor1.*r_com_mid.*t3;
t35 = l_top.*m_motor2.*r_com_mid.*t3;
t51 = t26+t27+t28;
t18 = cos(t16);
t20 = sin(t16);
t29 = r_com_bot+t12;
t30 = l_top.*t14;
t36 = m_bot.*r_com_bot.*t12.*2.0;
t37 = l_mid.*m_bot.*t13;
t38 = m_mid.*r_com_mid.*t13;
t39 = m_motor1.*r_com_mid.*t13;
t40 = m_motor2.*r_com_mid.*t13;
t41 = l_mid.*m_bot.*t15;
t42 = m_mid.*r_com_mid.*t15;
t43 = m_motor1.*r_com_mid.*t15;
t44 = m_motor2.*r_com_mid.*t15;
t45 = m_bot.*r_com_bot.*t30;
t46 = m_bot.*r_com_bot.*t18;
t47 = m_bot.*r_com_bot.*t20;
t48 = m_bot.*r_com_bot.*t29;
t49 = t29+t30;
t50 = m_bot.*r_com_bot.*t49;
t52 = t21+t22+t23+t24+t25+t31+t33+t34+t35+t36+t45;
M = reshape([t32,0.0,t17+t37+t38+t39+t40+t46+l_top.*m_bot.*t2+l_top.*m_mid.*t2+l_top.*m_motor1.*t2+l_top.*m_motor2.*t2,t37+t38+t39+t40+t46,t46,0.0,t32,t19+t41+t42+t43+t44+t47+l_top.*m_bot.*t5+l_top.*m_mid.*t5+l_top.*m_motor1.*t5+l_top.*m_motor2.*t5,t41+t42+t43+t44+t47,t47,t17+t51.*(l_top.*t2.*2.0+r_com_mid.*t13.*2.0)+m_bot.*(l_mid.*t13+l_top.*t2+r_com_bot.*t18),t19+t51.*(l_top.*t5.*2.0+r_com_mid.*t15.*2.0)+m_bot.*(l_mid.*t15+l_top.*t5+r_com_bot.*t20),I_bot+I_mid+I_top+t31+t33+t34+t35+t45+t52+m_bot.*t9+m_mid.*t9+m_motor1.*t9+m_motor2.*t9+m_top.*r_com_top.^2,t52,t50,(m_bot.*(l_mid.*t13.*2.0+r_com_bot.*t18.*2.0))./2.0+r_com_mid.*t13.*t51.*2.0,(m_bot.*(l_mid.*t15.*2.0+r_com_bot.*t20.*2.0))./2.0+r_com_mid.*t15.*t51.*2.0,t52,I_bot+I_mid+t21+t22+t23+t24+t25+t36,t48,t46,t47,t50,t48,I_bot+t22],[5,5]);
