function M = autogen_mass_matrix(I_bot,I_mid,I_top,l_mid,l_top,m_bot,m_mid,m_motor1,m_motor2,m_top,r_com_mid,r_com_bot,r_com_top,theta_2,theta_3)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(I_BOT,I_MID,I_TOP,L_MID,L_TOP,M_BOT,M_MID,M_MOTOR1,M_MOTOR2,M_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    07-May-2020 14:34:57

t2 = cos(theta_2);
t3 = cos(theta_3);
t4 = theta_2+theta_3;
t5 = l_mid.^2;
t6 = l_top.^2;
t7 = r_com_mid.^2;
t8 = r_com_bot.^2;
t9 = l_mid.*t3;
t10 = cos(t4);
t11 = m_bot.*t5;
t12 = m_bot.*t8;
t13 = m_mid.*t7;
t14 = m_motor1.*t7;
t15 = m_motor2.*t7;
t18 = l_mid.*l_top.*m_bot.*t2;
t19 = l_top.*m_mid.*r_com_mid.*t2;
t20 = l_top.*m_motor1.*r_com_mid.*t2;
t21 = l_top.*m_motor2.*r_com_mid.*t2;
t16 = r_com_bot+t9;
t17 = l_top.*t10;
t22 = m_bot.*r_com_bot.*t9.*2.0;
t23 = m_bot.*r_com_bot.*t17;
t24 = m_bot.*r_com_bot.*t16;
t25 = t16+t17;
t26 = m_bot.*r_com_bot.*t25;
t27 = t11+t12+t13+t14+t15+t18+t19+t20+t21+t22+t23;
M = reshape([I_top+t18+t19+t20+t21+t23+t27+m_bot.*t6+m_mid.*t6+m_motor1.*t6+m_motor2.*t6+m_top.*r_com_top.^2,t27,t26,t27,I_mid+t11+t12+t13+t14+t15+t22,t24,t26,t24,I_bot+t12],[3,3]);
