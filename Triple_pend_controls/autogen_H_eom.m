function H = autogen_H_eom(dtheta_1,dtheta_2,dtheta_3,g,l_mid,l_top,m_bot,m_mid,m_motor1,m_motor2,m_top,r_com_mid,r_com_bot,r_com_top,theta_1,theta_2,theta_3)
%AUTOGEN_H_EOM
%    H = AUTOGEN_H_EOM(DTHETA_1,DTHETA_2,DTHETA_3,G,L_MID,L_TOP,M_BOT,M_MID,M_MOTOR1,M_MOTOR2,M_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    25-May-2020 09:36:59

t2 = cos(theta_1);
t3 = sin(theta_1);
t4 = sin(theta_2);
t5 = sin(theta_3);
t6 = dtheta_1+dtheta_2;
t7 = theta_1+theta_2;
t8 = theta_2+theta_3;
t9 = dtheta_1.^2;
t10 = dtheta_2.^2;
t11 = dtheta_3.^2;
t19 = m_mid./2.0;
t20 = m_motor1./2.0;
t21 = m_motor2./2.0;
t12 = dtheta_3+t6;
t13 = cos(t7);
t14 = sin(t7);
t15 = sin(t8);
t16 = t7+theta_3;
t36 = l_mid.*m_bot.*r_com_bot.*t5.*t11;
t37 = dtheta_1.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t5.*2.0;
t38 = dtheta_2.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t5.*2.0;
t43 = t19+t20+t21;
t17 = cos(t16);
t18 = sin(t16);
t22 = dtheta_1.*l_mid.*t13;
t23 = dtheta_2.*l_mid.*t13;
t24 = dtheta_1.*l_mid.*t14;
t25 = dtheta_2.*l_mid.*t14;
t29 = g.*l_mid.*m_bot.*t14;
t30 = g.*m_mid.*r_com_mid.*t14;
t31 = g.*m_motor1.*r_com_mid.*t14;
t32 = g.*m_motor2.*r_com_mid.*t14;
t40 = -t37;
t41 = -t38;
t42 = -t36;
t26 = dtheta_1.*r_com_bot.*t17;
t27 = dtheta_2.*r_com_bot.*t17;
t28 = dtheta_3.*r_com_bot.*t17;
t33 = dtheta_1.*r_com_bot.*t18;
t34 = dtheta_2.*r_com_bot.*t18;
t35 = dtheta_3.*r_com_bot.*t18;
t39 = g.*m_bot.*r_com_bot.*t18;
H = [-dtheta_1.*(m_bot.*(t24+t25+t33+t34+t35+dtheta_1.*l_top.*t3)+t43.*(dtheta_1.*l_top.*t3.*2.0+dtheta_1.*r_com_mid.*t14.*2.0+dtheta_2.*r_com_mid.*t14.*2.0)+dtheta_1.*m_top.*r_com_top.*t3)-dtheta_2.*(m_bot.*(t24+t25+t33+t34+t35)+r_com_mid.*t6.*t14.*t43.*2.0)-m_bot.*t12.*t35;g.*(m_bot+m_mid+m_motor1+m_motor2+m_top)+dtheta_1.*(m_bot.*(t22+t23+t26+t27+t28+dtheta_1.*l_top.*t2)+t43.*(dtheta_1.*l_top.*t2.*2.0+dtheta_1.*r_com_mid.*t13.*2.0+dtheta_2.*r_com_mid.*t13.*2.0)+dtheta_1.*m_top.*r_com_top.*t2)+dtheta_2.*(m_bot.*(t22+t23+t26+t27+t28)+r_com_mid.*t6.*t13.*t43.*2.0)+m_bot.*t12.*t28;t29+t30+t31+t32+t39+t40+t41+t42+g.*l_top.*m_bot.*t3+g.*l_top.*m_mid.*t3+g.*l_top.*m_motor1.*t3+g.*l_top.*m_motor2.*t3+g.*m_top.*r_com_top.*t3-l_mid.*l_top.*m_bot.*t4.*t10-l_top.*m_bot.*r_com_bot.*t10.*t15-l_top.*m_bot.*r_com_bot.*t11.*t15-l_top.*m_mid.*r_com_mid.*t4.*t10-l_top.*m_motor1.*r_com_mid.*t4.*t10-l_top.*m_motor2.*r_com_mid.*t4.*t10-dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*t4.*2.0-dtheta_1.*dtheta_2.*l_top.*m_bot.*r_com_bot.*t15.*2.0-dtheta_1.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t15.*2.0-dtheta_2.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t15.*2.0-dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t4.*2.0-dtheta_1.*dtheta_2.*l_top.*m_motor1.*r_com_mid.*t4.*2.0-dtheta_1.*dtheta_2.*l_top.*m_motor2.*r_com_mid.*t4.*2.0;t29+t30+t31+t32+t39+t40+t41+t42+l_mid.*l_top.*m_bot.*t4.*t9+l_top.*m_bot.*r_com_bot.*t9.*t15+l_top.*m_mid.*r_com_mid.*t4.*t9+l_top.*m_motor1.*r_com_mid.*t4.*t9+l_top.*m_motor2.*r_com_mid.*t4.*t9;m_bot.*r_com_bot.*(g.*t18+l_mid.*t5.*t9+l_mid.*t5.*t10+l_top.*t9.*t15+dtheta_1.*dtheta_2.*l_mid.*t5.*2.0)];
