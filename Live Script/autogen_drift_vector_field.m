function f_ss = autogen_drift_vector_field(I_top, I_mid, I_bot , b1, b2, b3, dtheta_1, dtheta_2, dtheta_3, g, l_top, l_mid, l_bot, m_top, m_mid, m_bot, r_com_top,r_com_mid,r_com_bot,theta_1,theta_2,theta_3)
%AUTOGEN_DRIFT_VECTOR_FIELD
%    F_SS = AUTOGEN_DRIFT_VECTOR_FIELD(I_BOT,I_MID,I_TOP,B1,B2,B3,DTHETA_1,DTHETA_2,DTHETA_3,G,L_MID,L_TOP,M_BOT,M_MID,M_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    04-May-2020 15:22:02

t2 = cos(theta_1);
t3 = cos(theta_2);
t4 = cos(theta_3);
t5 = sin(theta_2);
t6 = sin(theta_3);
t7 = b1.*dtheta_1;
t8 = b2.*dtheta_2;
t9 = b3.*dtheta_3;
t10 = theta_1+theta_2;
t11 = theta_1+theta_3;
t12 = theta_2+theta_3;
t13 = dtheta_1.^2;
t14 = dtheta_2.^2;
t15 = dtheta_3.^2;
t16 = l_mid.^2;
t17 = l_mid.^3;
t18 = l_top.^2;
t19 = m_bot.^2;
t20 = m_mid.^2;
t21 = r_com_mid.^2;
t22 = r_com_mid.^3;
t23 = r_com_bot.^2;
t24 = r_com_bot.^3;
t25 = r_com_top.^2;
t26 = theta_2.*2.0;
t27 = theta_3.*2.0;
t28 = I_top.*r_com_bot.*2.0;
t38 = -theta_3;
t42 = I_bot.*I_mid.*I_top.*2.0;
t29 = cos(t26);
t30 = cos(t27);
t31 = sin(t26);
t32 = sin(t27);
t33 = cos(t10);
t34 = cos(t11);
t35 = cos(t12);
t36 = sin(t12);
t37 = t10+theta_3;
t39 = -t27;
t41 = -t7;
t43 = t10+theta_2;
t44 = t11+theta_3;
t45 = t12+theta_3;
t46 = t12+theta_2;
t47 = m_bot.*r_com_bot.*t18;
t54 = t10+t12;
t55 = g.*l_top.*m_bot.*t2;
t56 = g.*l_top.*m_mid.*t2;
t57 = g.*m_top.*r_com_top.*t2;
t58 = t38+theta_1;
t60 = t38+theta_2;
t61 = I_top.*l_mid.*t4.*2.0;
t62 = t12.*2.0;
t63 = m_mid.*r_com_bot.*t18.*2.0;
t67 = m_top.*r_com_bot.*t25.*2.0;
t70 = I_bot.*m_bot.*t16.*2.0;
t71 = I_bot.*m_bot.*t23.*2.0;
t72 = I_bot.*m_mid.*t21.*2.0;
t73 = I_top.*m_bot.*t23.*2.0;
t74 = l_mid.*l_top.*m_bot.*r_com_bot.*t3;
t76 = I_bot.*I_mid.*m_bot.*t18.*2.0;
t78 = I_bot.*I_mid.*m_mid.*t18.*2.0;
t84 = I_bot.*I_mid.*m_top.*t25.*2.0;
t93 = I_bot.*l_mid.*m_bot.*r_com_bot.*t4.*4.0;
t94 = l_mid.*m_bot.*t4.*t18;
t95 = m_mid.*r_com_mid.*t4.*t18;
t96 = l_top.*m_mid.*r_com_mid.*r_com_bot.*t3.*2.0;
t97 = I_bot.*I_mid.*l_mid.*l_top.*m_bot.*t3.*4.0;
t100 = I_bot.*I_mid.*l_top.*m_mid.*r_com_mid.*t3.*4.0;
t101 = dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*t5.*4.0;
t102 = dtheta_1.*dtheta_2.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t103 = dtheta_1.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t104 = dtheta_2.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t105 = dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t5.*4.0;
t108 = l_mid.*m_mid.*t4.*t18.*2.0;
t110 = l_mid.*m_top.*t4.*t25.*2.0;
t111 = I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t18.*2.0;
t113 = l_mid.*l_top.*m_bot.*t5.*t13.*2.0;
t114 = l_mid.*l_top.*m_bot.*t5.*t14.*2.0;
t116 = l_mid.*m_bot.*r_com_bot.*t6.*t13.*2.0;
t117 = l_mid.*m_bot.*r_com_bot.*t6.*t14.*2.0;
t118 = l_mid.*m_bot.*r_com_bot.*t6.*t15.*2.0;
t119 = l_top.*m_mid.*r_com_mid.*t5.*t13.*2.0;
t120 = l_top.*m_mid.*r_com_mid.*t5.*t14.*2.0;
t124 = t16.*t19.*t23;
t125 = t18.*t19.*t23;
t126 = m_bot.*m_mid.*t18.*t23.*2.0;
t127 = m_bot.*m_mid.*t21.*t23.*2.0;
t128 = m_bot.*m_top.*t23.*t25.*2.0;
t130 = I_bot.*t16.*t18.*t19;
t133 = I_bot.*t18.*t20.*t21;
t158 = I_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*t3.*t23.*4.0;
t171 = I_bot.*l_mid.*r_com_bot.*t4.*t18.*t19.*2.0;
t172 = I_mid.*l_mid.*l_top.*t3.*t19.*t23.*2.0;
t175 = m_bot.*t18.*t20.*t21.*t23;
t40 = cos(t37);
t48 = cos(t43);
t49 = cos(t44);
t50 = cos(t45);
t51 = cos(t46);
t52 = sin(t45);
t53 = sin(t46);
t59 = t39+theta_1;
t64 = cos(t58);
t66 = cos(t60);
t68 = sin(t60);
t69 = cos(t54);
t75 = I_mid.*t70;
t77 = I_top.*t70;
t79 = I_mid.*t71;
t80 = I_top.*t71;
t81 = I_mid.*t72;
t82 = I_top.*t72;
t83 = I_mid.*t73;
t85 = g.*l_mid.*m_bot.*t33;
t86 = g.*m_mid.*r_com_mid.*t33;
t87 = cos(t62);
t88 = -t55;
t89 = -t56;
t90 = sin(t62);
t91 = t12+t37;
t92 = -t57;
t98 = I_mid.*t93;
t99 = I_top.*t93;
t106 = l_mid.*l_top.*m_mid.*r_com_mid.*t35;
t121 = -t103;
t122 = -t104;
t123 = l_top.*m_bot.*t16.*t35;
t129 = -t95;
t131 = I_bot.*t125;
t132 = I_mid.*t124;
t134 = I_mid.*t125;
t135 = I_top.*t124;
t136 = m_mid.*t18.*t70;
t137 = m_bot.*t18.*t72;
t138 = m_mid.*t18.*t71;
t139 = m_top.*t25.*t70;
t140 = I_mid.*t126;
t142 = m_top.*t25.*t71;
t143 = I_mid.*t127;
t144 = m_mid.*t21.*t73;
t145 = m_top.*t25.*t72;
t146 = I_mid.*t128;
t147 = -t111;
t148 = dtheta_1.*dtheta_2.*l_top.*m_bot.*r_com_bot.*t36.*4.0;
t149 = dtheta_1.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t36.*4.0;
t150 = dtheta_2.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t36.*4.0;
t151 = -t118;
t152 = l_top.*m_mid.*t21.*t35.*2.0;
t154 = I_bot.*I_mid.*l_top.*m_bot.*r_com_bot.*t35.*4.0;
t155 = I_bot.*l_mid.*m_mid.*t4.*t47.*4.0;
t156 = I_bot.*m_mid.*r_com_mid.*t4.*t47.*2.0;
t157 = m_top.*t25.*t93;
t162 = l_top.*m_bot.*r_com_bot.*t13.*t36.*2.0;
t163 = l_top.*m_bot.*r_com_bot.*t14.*t36.*2.0;
t164 = l_top.*m_bot.*r_com_bot.*t15.*t36.*2.0;
t169 = l_mid.*m_mid.*r_com_mid.*t125;
t173 = t30.*t124;
t174 = m_mid.*t18.*t124;
t176 = m_mid.*t21.*t125;
t177 = m_top.*t25.*t124;
t180 = m_top.*t25.*t127;
t181 = t29.*t111;
t184 = t29.*t130;
t186 = t29.*t133;
t188 = I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t18.*t29.*-2.0;
t205 = t29.*t175;
t65 = cos(t59);
t107 = g.*m_bot.*r_com_bot.*t40;
t109 = cos(t91);
t112 = -t85;
t115 = -t86;
t141 = l_mid.*l_top.*m_bot.*r_com_bot.*t50;
t159 = l_mid.*m_bot.*t18.*t51;
t160 = m_mid.*r_com_mid.*t18.*t51;
t161 = l_mid.*l_top.*m_mid.*r_com_mid.*t66;
t165 = -t123;
t166 = -t152;
t167 = -t156;
t168 = l_top.*m_bot.*t16.*t66;
t182 = t47.*t87;
t183 = -t169;
t185 = t30.*t132;
t187 = t30.*t135;
t189 = -t173;
t190 = I_bot.*m_mid.*r_com_mid.*t47.*t51.*2.0;
t192 = I_bot.*l_mid.*r_com_bot.*t18.*t19.*t51.*2.0;
t193 = I_mid.*l_mid.*l_top.*t19.*t23.*t50.*2.0;
t195 = -t184;
t197 = -t186;
t199 = t29.*t169;
t200 = t30.*t169;
t203 = t87.*t125;
t204 = m_mid.*t18.*t173;
t206 = m_top.*t25.*t173;
t208 = t87.*t131;
t209 = t87.*t134;
t212 = -t205;
t216 = t87.*t169;
t217 = t87.*t176;
t153 = -t107;
t170 = -t141;
t178 = -t159;
t179 = -t160;
t191 = -t182;
t194 = -t190;
t196 = -t185;
t198 = -t187;
t201 = -t192;
t202 = -t193;
t207 = t29.*t183;
t210 = -t203;
t211 = m_mid.*t18.*t189;
t213 = m_top.*t25.*t189;
t214 = -t208;
t215 = -t209;
t218 = -t217;
t219 = t9+t102+t107+t116+t117+t162;
t220 = t8+t85+t86+t107+t113+t119+t121+t122+t151+t162;
t221 = t41+t88+t89+t92+t101+t103+t104+t105+t112+t114+t115+t118+t120+t148+t149+t150+t153+t163+t164;
t222 = t28+t47+t61+t63+t67+t74+t94+t96+t106+t108+t110+t129+t161+t165+t166+t168+t170+t178+t179+t191;
t223 = t42+t75+t76+t77+t78+t79+t80+t81+t82+t83+t84+t97+t98+t99+t100+t130+t131+t132+t133+t134+t135+t136+t137+t138+t139+t140+t142+t143+t144+t145+t146+t147+t154+t155+t157+t158+t167+t171+t172+t174+t175+t176+t177+t180+t183+t188+t194+t195+t196+t197+t198+t200+t201+t202+t207+t211+t212+t213+t214+t215+t216+t218;
t224 = 1.0./t223;
f_ss = [dtheta_1;dtheta_2;dtheta_3;t224.*(t7.*t124.*(-1.0./2.0)+(t8.*t124)./2.0+(t7.*t173)./2.0-(t56.*t124)./2.0-(t8.*t173)./2.0-(t57.*t124)./2.0+I_bot.*I_mid.*t41+I_bot.*I_mid.*t88+I_bot.*I_mid.*t89+I_bot.*I_mid.*t92+I_bot.*I_mid.*t101+I_bot.*I_mid.*t103+I_bot.*I_mid.*t104+I_bot.*I_mid.*t105+I_bot.*I_mid.*t112+I_bot.*I_mid.*t114+I_bot.*I_mid.*t115+I_bot.*I_mid.*t118+I_bot.*I_mid.*t120+I_bot.*I_mid.*t148+I_bot.*I_mid.*t149+I_bot.*I_mid.*t150+I_bot.*I_mid.*t153+I_bot.*I_mid.*t163+I_bot.*I_mid.*t164+t13.*t31.*t111+t13.*t31.*t130+t13.*t31.*t133+t13.*t32.*t132+t14.*t32.*t132+t13.*t31.*t169+t13.*t31.*t175+t13.*t90.*t131+t13.*t90.*t134+t13.*t90.*t176+t13.*t90.*t183+dtheta_1.*dtheta_2.*t32.*t132.*2.0-(m_bot.*r_com_bot.*t9.*t106)./2.0-(m_bot.*r_com_bot.*t9.*t161)./2.0-(t19.*t21.*t23.*t56)./2.0-(t20.*t21.*t23.*t55)./2.0+I_bot.*m_bot.*t8.*t16+I_bot.*m_bot.*t8.*t23+I_bot.*m_bot.*t16.*t41+I_bot.*m_bot.*t23.*t41+I_bot.*m_bot.*t16.*t92+I_bot.*m_bot.*t23.*t92+I_bot.*m_bot.*t16.*t105+I_bot.*m_bot.*t23.*t105+I_bot.*m_mid.*t8.*t21+I_mid.*m_bot.*t9.*t23+I_bot.*m_mid.*t21.*t41+I_mid.*m_bot.*t23.*t41+I_bot.*m_mid.*t16.*t88+I_bot.*m_mid.*t21.*t88+I_bot.*m_mid.*t23.*t88+I_bot.*m_mid.*t21.*t92+I_mid.*m_bot.*t23.*t92+I_bot.*m_mid.*t21.*t101+I_mid.*m_bot.*t23.*t105+I_mid.*m_bot.*t23.*t115+I_mid.*m_bot.*t23.*t120+I_bot.*m_mid.*t21.*t148+I_bot.*m_mid.*t21.*t149+I_bot.*m_mid.*t21.*t150+I_mid.*m_mid.*t23.*t88+I_bot.*l_mid.*m_mid.*r_com_mid.*t55+I_bot.*l_mid.*m_mid.*r_com_mid.*t148+I_bot.*l_mid.*m_mid.*r_com_mid.*t162+I_bot.*l_mid.*m_mid.*r_com_mid.*t163+(g.*l_top.*m_mid.*t49.*t124)./4.0+(g.*l_top.*m_mid.*t65.*t124)./4.0+(g.*m_top.*r_com_top.*t49.*t124)./4.0+(g.*m_top.*r_com_top.*t65.*t124)./4.0+m_bot.*m_mid.*t8.*t21.*t23+m_bot.*m_mid.*t21.*t23.*t41+m_bot.*m_mid.*t21.*t23.*t92+(l_mid.*r_com_mid.*t19.*t23.*t56)./2.0+l_mid.*l_top.*m_bot.*t5.*t13.*t72+l_mid.*l_top.*m_bot.*t5.*t14.*t72+l_top.*m_bot.*r_com_bot.*t13.*t36.*t72+l_top.*m_bot.*r_com_bot.*t14.*t36.*t72+l_top.*m_bot.*r_com_bot.*t15.*t36.*t72+l_top.*m_mid.*r_com_mid.*t5.*t13.*t70+l_top.*m_mid.*r_com_mid.*t5.*t13.*t71+l_top.*m_mid.*r_com_mid.*t5.*t14.*t70+l_top.*m_mid.*r_com_mid.*t5.*t14.*t71+l_top.*m_mid.*r_com_mid.*t5.*t13.*t124+l_top.*m_mid.*r_com_mid.*t5.*t14.*t124-l_top.*m_mid.*r_com_mid.*t13.*t52.*t124-l_top.*m_mid.*r_com_mid.*t14.*t52.*t124+(l_mid.*l_top.*t3.*t8.*t19.*t23)./2.0-(l_mid.*l_top.*t3.*t9.*t19.*t23)./2.0-(l_mid.*l_top.*t8.*t19.*t23.*t50)./2.0+(l_mid.*l_top.*t9.*t19.*t23.*t50)./2.0+(l_top.*r_com_bot.*t9.*t16.*t19.*t35)./2.0-(l_top.*r_com_bot.*t9.*t16.*t19.*t66)./2.0+(I_mid.*g.*l_mid.*t19.*t23.*cos(t10+t27))./2.0+I_bot.*l_mid.*l_top.*m_bot.*t3.*t8-(I_bot.*g.*l_top.*t2.*t16.*t19)./2.0-(I_bot.*g.*l_top.*t2.*t20.*t21)./2.0-(I_bot.*g.*l_top.*t2.*t19.*t23)./2.0+(I_bot.*g.*l_top.*t16.*t19.*t48)./2.0+(I_bot.*g.*l_top.*t20.*t21.*t48)./2.0+(I_bot.*g.*l_top.*t19.*t23.*t109)./2.0-(I_mid.*g.*l_mid.*t19.*t23.*t33)./2.0-(I_mid.*g.*l_top.*t2.*t19.*t23)./2.0+(I_mid.*g.*l_top.*t19.*t23.*t109)./2.0-I_bot.*l_mid.*m_bot.*r_com_bot.*t4.*t7.*2.0+I_bot.*l_mid.*m_bot.*r_com_bot.*t4.*t8.*2.0+I_bot.*l_top.*m_bot.*r_com_bot.*t8.*t35+I_mid.*l_mid.*m_bot.*r_com_bot.*t4.*t9+I_bot.*l_top.*m_mid.*r_com_mid.*t3.*t8+I_mid.*l_top.*m_bot.*r_com_bot.*t9.*t35+I_bot.*m_mid.*r_com_mid.*t13.*t47.*t53.*2.0+I_bot.*l_top.*t5.*t13.*t17.*t19.*2.0+I_bot.*l_top.*t5.*t14.*t17.*t19.*2.0+I_bot.*l_top.*t5.*t13.*t20.*t22.*2.0+I_bot.*l_top.*t5.*t14.*t20.*t22.*2.0+I_bot.*l_top.*t13.*t19.*t24.*t36.*2.0+I_bot.*l_top.*t14.*t19.*t24.*t36.*2.0+I_bot.*l_top.*t15.*t19.*t24.*t36.*2.0+I_mid.*l_mid.*t6.*t13.*t19.*t24.*2.0+I_mid.*l_mid.*t6.*t14.*t19.*t24.*2.0+I_mid.*l_mid.*t6.*t15.*t19.*t24.*2.0+I_mid.*l_top.*t13.*t19.*t24.*t36.*2.0+I_mid.*l_top.*t14.*t19.*t24.*t36.*2.0+I_mid.*l_top.*t15.*t19.*t24.*t36.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t17.*t19.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t20.*t22.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t19.*t24.*t36.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*t19.*t24.*t36.*4.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*t19.*t24.*t36.*4.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*t6.*t19.*t24.*4.0+I_mid.*dtheta_1.*dtheta_3.*l_mid.*t6.*t19.*t24.*4.0+I_mid.*dtheta_2.*dtheta_3.*l_mid.*t6.*t19.*t24.*4.0+I_mid.*dtheta_1.*dtheta_2.*l_top.*t19.*t24.*t36.*4.0+I_mid.*dtheta_1.*dtheta_3.*l_top.*t19.*t24.*t36.*4.0+I_mid.*dtheta_2.*dtheta_3.*l_top.*t19.*t24.*t36.*4.0-(I_bot.*g.*l_mid.*l_top.*r_com_bot.*t19.*t34)./2.0-(I_bot.*g.*l_mid.*l_top.*r_com_bot.*t19.*t64)./2.0+I_bot.*g.*l_mid.*l_top.*r_com_bot.*t19.*t69+I_bot.*l_mid.*l_top.*t5.*t13.*t19.*t23.*4.0+I_bot.*l_mid.*l_top.*t5.*t14.*t19.*t23.*4.0+I_bot.*l_mid.*l_top.*t5.*t15.*t19.*t23.*3.0+I_bot.*l_mid.*l_top.*t13.*t19.*t23.*t52.*2.0+I_bot.*l_mid.*l_top.*t14.*t19.*t23.*t52.*2.0+I_bot.*l_mid.*l_top.*t15.*t19.*t23.*t52+I_mid.*l_mid.*l_top.*t5.*t14.*t19.*t23+I_mid.*l_mid.*l_top.*t13.*t19.*t23.*t52.*2.0+I_mid.*l_mid.*l_top.*t14.*t19.*t23.*t52+I_bot.*l_mid.*r_com_bot.*t13.*t18.*t19.*t53.*2.0+I_bot.*l_top.*r_com_bot.*t13.*t16.*t19.*t36.*4.0+I_bot.*l_top.*r_com_bot.*t14.*t16.*t19.*t36.*4.0+I_bot.*l_top.*r_com_bot.*t15.*t16.*t19.*t36+I_bot.*l_top.*r_com_bot.*t13.*t16.*t19.*t68.*2.0+I_bot.*l_top.*r_com_bot.*t14.*t16.*t19.*t68.*2.0+I_bot.*l_top.*r_com_bot.*t15.*t16.*t19.*t68+dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t5.*t124.*2.0-dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t52.*t124.*2.0+(g.*l_top.*m_bot.*t20.*t21.*t23.*t48)./2.0+(g.*l_top.*m_mid.*t19.*t21.*t23.*t109)./2.0+l_top.*m_bot.*m_mid.*r_com_mid.*t3.*t8.*t23-l_top.*m_bot.*m_mid.*r_com_mid.*t3.*t9.*t23+l_top.*m_bot.*m_mid.*r_com_bot.*t9.*t21.*t35+l_top.*m_bot.*t5.*t13.*t20.*t22.*t23.*2.0+l_top.*m_bot.*t5.*t14.*t20.*t22.*t23.*2.0+l_top.*m_mid.*t13.*t19.*t21.*t24.*t36.*2.0+l_top.*m_mid.*t14.*t19.*t21.*t24.*t36.*2.0+l_top.*m_mid.*t15.*t19.*t21.*t24.*t36.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t5.*t19.*t23.*8.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*t5.*t19.*t23.*6.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*t5.*t19.*t23.*6.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t19.*t23.*t52.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*t19.*t23.*t52.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*t19.*t23.*t52.*2.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t5.*t19.*t23.*2.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t19.*t23.*t52.*2.0-I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_bot.*t34+I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*t48-I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_bot.*t64+I_bot.*dtheta_1.*dtheta_2.*l_top.*r_com_bot.*t16.*t19.*t36.*8.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*r_com_bot.*t16.*t19.*t36.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*r_com_bot.*t16.*t19.*t36.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*r_com_bot.*t16.*t19.*t68.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*r_com_bot.*t16.*t19.*t68.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*r_com_bot.*t16.*t19.*t68.*2.0+(I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t34)./2.0-I_bot.*g.*l_mid.*m_bot.*m_top.*r_com_bot.*r_com_top.*t34+(I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t64)./2.0+I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t69-I_bot.*g.*l_mid.*m_bot.*m_top.*r_com_bot.*r_com_top.*t64+dtheta_1.*dtheta_2.*l_top.*m_bot.*t5.*t20.*t22.*t23.*4.0+dtheta_1.*dtheta_2.*l_top.*m_mid.*t19.*t21.*t24.*t36.*4.0+dtheta_1.*dtheta_3.*l_top.*m_mid.*t19.*t21.*t24.*t36.*4.0+dtheta_2.*dtheta_3.*l_top.*m_mid.*t19.*t21.*t24.*t36.*4.0+(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t23.*t48)./2.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t23.*t49)./4.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t23.*t65)./4.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t23.*t109)./2.0-l_mid.*l_top.*m_mid.*r_com_mid.*t13.*t19.*t24.*t36-l_mid.*l_top.*m_mid.*r_com_mid.*t14.*t19.*t24.*t36-l_mid.*l_top.*m_mid.*r_com_mid.*t15.*t19.*t24.*t36+l_mid.*l_top.*m_mid.*r_com_mid.*t13.*t19.*t24.*t68+l_mid.*l_top.*m_mid.*r_com_mid.*t14.*t19.*t24.*t68+l_mid.*l_top.*m_mid.*r_com_mid.*t15.*t19.*t24.*t68+l_mid.*l_top.*m_mid.*t5.*t13.*t19.*t21.*t23+l_mid.*l_top.*m_mid.*t5.*t14.*t19.*t21.*t23+l_mid.*l_top.*m_mid.*t13.*t19.*t21.*t23.*t52+l_mid.*l_top.*m_mid.*t14.*t19.*t21.*t23.*t52-I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t15.*t36+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t13.*t68.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t14.*t68.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t15.*t68-dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t36.*2.0-dtheta_1.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t36.*2.0-dtheta_2.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t36.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t68.*2.0+dtheta_1.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t68.*2.0+dtheta_2.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t19.*t24.*t68.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*t5.*t19.*t21.*t23.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*t19.*t21.*t23.*t52.*2.0-I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t36.*2.0-I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t36.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t68.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t68.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t68.*2.0).*2.0;-t220.*t224.*(t70+t71+t72+t73+t93+t124+t125+t126+t127+t128+t189+t210+I_bot.*I_top.*2.0+I_bot.*m_bot.*t18.*2.0+I_bot.*m_mid.*t18.*2.0+I_bot.*m_top.*t25.*2.0+I_bot.*l_mid.*l_top.*m_bot.*t3.*4.0+I_bot.*l_top.*m_bot.*r_com_bot.*t35.*4.0+I_bot.*l_top.*m_mid.*r_com_mid.*t3.*4.0+l_mid.*l_top.*t3.*t19.*t23.*2.0-l_mid.*l_top.*t19.*t23.*t50.*2.0+l_top.*m_bot.*m_mid.*r_com_mid.*t3.*t23.*4.0)-t221.*t224.*(t70+t71+t72+t93+t124+t127+t189+I_bot.*l_mid.*l_top.*m_bot.*t3.*2.0+I_bot.*l_top.*m_bot.*r_com_bot.*t35.*2.0+I_bot.*l_top.*m_mid.*r_com_mid.*t3.*2.0+l_mid.*l_top.*t3.*t19.*t23-l_mid.*l_top.*t19.*t23.*t50+l_top.*m_bot.*m_mid.*r_com_mid.*t3.*t23.*2.0)+m_bot.*r_com_bot.*t219.*t222.*t224;-t219.*t224.*(t73+t125+t126+t128+t210+I_mid.*I_top.*2.0+I_mid.*m_bot.*t16.*2.0+I_mid.*m_bot.*t18.*2.0+I_mid.*m_bot.*t23.*2.0+I_top.*m_bot.*t16.*2.0+I_mid.*m_mid.*t18.*2.0+I_mid.*m_mid.*t21.*2.0+I_top.*m_mid.*t21.*2.0+I_mid.*m_top.*t25.*2.0+t16.*t18.*t19+t18.*t20.*t21+l_mid.*m_mid.*t4.*t47.*4.0+m_bot.*m_mid.*t16.*t18.*2.0+m_bot.*m_mid.*t18.*t21.*2.0+m_bot.*m_top.*t16.*t25.*2.0+m_mid.*m_top.*t21.*t25.*2.0-m_mid.*r_com_mid.*t4.*t47.*2.0-m_mid.*r_com_mid.*t47.*t51.*2.0-t16.*t18.*t19.*t29-t18.*t20.*t21.*t29+I_mid.*l_mid.*l_top.*m_bot.*t3.*4.0+I_mid.*l_mid.*m_bot.*r_com_bot.*t4.*4.0+I_top.*l_mid.*m_bot.*r_com_bot.*t4.*4.0+I_mid.*l_top.*m_bot.*r_com_bot.*t35.*4.0+I_mid.*l_top.*m_mid.*r_com_mid.*t3.*4.0-l_mid.*m_bot.*m_mid.*r_com_mid.*t18.*2.0+l_mid.*r_com_bot.*t4.*t18.*t19.*2.0-l_mid.*r_com_bot.*t18.*t19.*t51.*2.0-l_mid.*m_bot.*m_mid.*r_com_mid.*t18.*t29.*2.0+l_mid.*m_bot.*m_top.*r_com_bot.*t4.*t25.*4.0)-m_bot.*r_com_bot.*t221.*t224.*(-t74-t96-t106+t123+t141+t152-t161-t168+I_mid.*r_com_bot.*2.0+I_mid.*l_mid.*t4.*2.0+I_mid.*l_top.*t35.*2.0)+m_bot.*r_com_bot.*t220.*t222.*t224];
