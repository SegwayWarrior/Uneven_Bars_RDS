function f_ss = autogen_drift_vector_field(I_bot,I_mid,I_top,b1,b2,b3,dtheta_1,dtheta_2,dtheta_3,g,l_mid,l_top,m_bot,m_mid,m_motor1,m_motor2,m_top,r_com_mid,r_com_bot,r_com_top,theta_1,theta_2,theta_3)
%AUTOGEN_DRIFT_VECTOR_FIELD
%    F_SS = AUTOGEN_DRIFT_VECTOR_FIELD(I_BOT,I_MID,I_TOP,B1,B2,B3,DTHETA_1,DTHETA_2,DTHETA_3,G,L_MID,L_TOP,M_BOT,M_MID,M_MOTOR1,M_MOTOR2,M_TOP,R_COM_MID,R_COM_BOT,R_COM_TOP,THETA_1,THETA_2,THETA_3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    07-May-2020 14:36:06

t2 = cos(theta_2);
t3 = cos(theta_3);
t4 = sin(theta_1);
t5 = sin(theta_2);
t6 = sin(theta_3);
t7 = I_top.*r_com_bot;
t8 = b1.*dtheta_1;
t9 = b2.*dtheta_2;
t10 = b3.*dtheta_3;
t11 = theta_1+theta_2;
t12 = theta_1+theta_3;
t13 = theta_2+theta_3;
t14 = dtheta_1.^2;
t15 = dtheta_2.^2;
t16 = dtheta_3.^2;
t17 = l_mid.^2;
t18 = l_mid.^3;
t19 = l_top.^2;
t20 = m_bot.^2;
t21 = m_bot.^3;
t22 = m_mid.^2;
t23 = m_motor1.^2;
t24 = m_motor2.^2;
t25 = r_com_mid.^2;
t26 = r_com_mid.^3;
t27 = r_com_bot.^2;
t28 = r_com_bot.^3;
t29 = r_com_top.^2;
t30 = theta_2.*2.0;
t31 = theta_3.*2.0;
t36 = I_bot.*I_mid.*I_top;
t44 = -theta_3;
t32 = cos(t30);
t33 = cos(t31);
t34 = t2.^2;
t35 = t3.^2;
t37 = sin(t30);
t38 = sin(t31);
t39 = cos(t13);
t40 = sin(t11);
t41 = sin(t12);
t42 = sin(t13);
t43 = t11+theta_3;
t45 = -t31;
t46 = -t8;
t48 = t11+theta_2;
t49 = t12+theta_3;
t50 = t13+theta_3;
t51 = t13+theta_2;
t52 = I_top.*l_mid.*t3;
t53 = m_bot.*r_com_bot.*t19;
t54 = m_mid.*r_com_bot.*t19;
t55 = m_motor1.*r_com_bot.*t19;
t56 = m_motor2.*r_com_bot.*t19;
t60 = m_top.*r_com_bot.*t29;
t65 = t11+t13;
t66 = g.*l_top.*m_bot.*t4;
t67 = g.*l_top.*m_mid.*t4;
t68 = g.*l_top.*m_motor1.*t4;
t69 = g.*l_top.*m_motor2.*t4;
t70 = g.*m_top.*r_com_top.*t4;
t71 = t44+theta_1;
t73 = t44+theta_2;
t74 = I_bot.*m_bot.*t17;
t75 = I_bot.*m_bot.*t27;
t76 = I_bot.*m_mid.*t25;
t77 = I_bot.*m_motor1.*t25;
t78 = I_bot.*m_motor2.*t25;
t79 = m_bot.*r_com_bot.*t7;
t80 = t13.*2.0;
t86 = I_bot.*I_mid.*m_bot.*t19;
t88 = I_bot.*I_mid.*m_mid.*t19;
t89 = I_bot.*I_mid.*m_motor1.*t19;
t90 = I_bot.*I_mid.*m_motor2.*t19;
t98 = I_bot.*I_mid.*m_top.*t29;
t102 = l_mid.*l_top.*m_bot.*r_com_bot.*t2;
t103 = l_top.*m_mid.*r_com_mid.*r_com_bot.*t2;
t104 = l_top.*m_motor1.*r_com_mid.*r_com_bot.*t2;
t105 = l_top.*m_motor2.*r_com_mid.*r_com_bot.*t2;
t118 = I_bot.*l_mid.*m_bot.*r_com_bot.*t3.*2.0;
t119 = l_mid.*m_bot.*t3.*t19;
t120 = l_mid.*m_mid.*t3.*t19;
t121 = l_mid.*m_motor1.*t3.*t19;
t122 = l_mid.*m_motor2.*t3.*t19;
t123 = l_mid.*m_top.*t3.*t29;
t124 = I_bot.*I_mid.*l_mid.*l_top.*m_bot.*t2.*2.0;
t126 = I_bot.*l_mid.*m_bot.*t3.*t7.*2.0;
t127 = I_bot.*I_mid.*l_top.*m_mid.*r_com_mid.*t2.*2.0;
t128 = I_bot.*I_mid.*l_top.*m_motor1.*r_com_mid.*t2.*2.0;
t129 = I_bot.*I_mid.*l_top.*m_motor2.*r_com_mid.*t2.*2.0;
t130 = dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*t5.*4.0;
t131 = dtheta_1.*dtheta_2.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t132 = dtheta_1.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t133 = dtheta_2.*dtheta_3.*l_mid.*m_bot.*r_com_bot.*t6.*4.0;
t134 = dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t5.*4.0;
t135 = dtheta_1.*dtheta_2.*l_top.*m_motor1.*r_com_mid.*t5.*4.0;
t136 = dtheta_1.*dtheta_2.*l_top.*m_motor2.*r_com_mid.*t5.*4.0;
t137 = l_mid.*l_top.*m_mid.*r_com_mid.*t2.*t3;
t138 = l_mid.*l_top.*m_motor1.*r_com_mid.*t2.*t3;
t139 = l_mid.*l_top.*m_motor2.*r_com_mid.*t2.*t3;
t141 = m_bot.*m_mid.*t19.*t27;
t142 = m_bot.*m_motor1.*t19.*t27;
t143 = m_bot.*m_motor2.*t19.*t27;
t144 = m_bot.*m_mid.*t25.*t27;
t145 = m_bot.*m_motor1.*t25.*t27;
t146 = m_bot.*m_motor2.*t25.*t27;
t147 = m_bot.*m_top.*t27.*t29;
t173 = l_mid.*l_top.*m_bot.*t5.*t14.*2.0;
t174 = l_mid.*l_top.*m_bot.*t5.*t15.*2.0;
t175 = l_mid.*m_bot.*r_com_bot.*t6.*t14.*2.0;
t176 = l_mid.*m_bot.*r_com_bot.*t6.*t15.*2.0;
t177 = l_mid.*m_bot.*r_com_bot.*t6.*t16.*2.0;
t178 = l_top.*m_mid.*r_com_mid.*t5.*t14.*2.0;
t179 = l_top.*m_mid.*r_com_mid.*t5.*t15.*2.0;
t180 = l_top.*m_motor1.*r_com_mid.*t5.*t14.*2.0;
t181 = l_top.*m_motor2.*r_com_mid.*t5.*t14.*2.0;
t182 = l_top.*m_motor1.*r_com_mid.*t5.*t15.*2.0;
t183 = l_top.*m_motor2.*r_com_mid.*t5.*t15.*2.0;
t191 = l_top.*m_bot.*t2.*t3.*t17;
t195 = t17.*t20.*t27;
t196 = t19.*t20.*t27;
t197 = I_bot.*t17.*t19.*t20;
t200 = I_bot.*t19.*t22.*t25;
t202 = r_com_bot.*t7.*t17.*t20;
t203 = I_bot.*t19.*t23.*t25;
t204 = I_bot.*t19.*t24.*t25;
t219 = I_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t27.*2.0;
t220 = I_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t27.*2.0;
t221 = I_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t27.*2.0;
t237 = t17.*t19.*t21.*t27;
t239 = I_bot.*l_mid.*r_com_bot.*t3.*t19.*t20.*2.0;
t240 = I_mid.*l_mid.*l_top.*t2.*t20.*t27.*2.0;
t245 = m_bot.*t19.*t22.*t25.*t27;
t248 = m_bot.*t19.*t23.*t25.*t27;
t250 = m_bot.*t19.*t24.*t25.*t27;
t47 = sin(t43);
t57 = cos(t50);
t58 = cos(t51);
t59 = t39.^2;
t61 = sin(t48);
t62 = sin(t49);
t63 = sin(t50);
t64 = sin(t51);
t72 = t45+theta_1;
t81 = cos(t73);
t82 = sin(t71);
t84 = sin(t73);
t85 = I_mid.*t74;
t87 = I_top.*t74;
t91 = I_mid.*t75;
t92 = I_bot.*t79;
t93 = I_mid.*t76;
t94 = I_mid.*t77;
t95 = I_mid.*t78;
t96 = I_top.*t76;
t97 = I_mid.*t79;
t99 = I_top.*t77;
t100 = I_top.*t78;
t101 = sin(t65);
t106 = g.*l_mid.*m_bot.*t40;
t107 = g.*m_mid.*r_com_mid.*t40;
t108 = g.*m_motor1.*r_com_mid.*t40;
t109 = g.*m_motor2.*r_com_mid.*t40;
t110 = cos(t80);
t111 = sin(t80);
t112 = t13+t43;
t113 = -t66;
t114 = -t67;
t115 = -t68;
t116 = -t69;
t117 = -t70;
t125 = I_mid.*t118;
t149 = m_mid.*t19.*t74;
t150 = m_motor1.*t19.*t74;
t151 = m_motor2.*t19.*t74;
t152 = m_bot.*t19.*t76;
t153 = m_mid.*t19.*t75;
t154 = m_top.*t29.*t74;
t155 = m_bot.*t19.*t77;
t156 = m_bot.*t19.*t78;
t157 = m_motor1.*t19.*t75;
t158 = m_motor2.*t19.*t75;
t159 = I_mid.*t141;
t160 = I_mid.*t142;
t161 = I_mid.*t143;
t162 = m_top.*t29.*t75;
t163 = I_mid.*t144;
t164 = I_mid.*t145;
t165 = I_mid.*t146;
t166 = m_mid.*t25.*t79;
t167 = m_top.*t29.*t76;
t168 = I_mid.*t147;
t169 = m_motor1.*t25.*t79;
t170 = m_motor2.*t25.*t79;
t171 = m_top.*t29.*t77;
t172 = m_top.*t29.*t78;
t185 = -t132;
t186 = -t133;
t190 = l_top.*m_bot.*t17.*t39;
t192 = l_top.*m_mid.*t25.*t39;
t193 = l_top.*m_motor1.*t25.*t39;
t194 = l_top.*m_motor2.*t25.*t39;
t198 = I_bot.*t196;
t199 = I_mid.*t195;
t201 = I_mid.*t196;
t205 = m_motor1.*t19.*t76.*2.0;
t206 = m_motor2.*t19.*t76.*2.0;
t207 = m_motor2.*t19.*t77.*2.0;
t208 = dtheta_1.*dtheta_2.*l_top.*m_bot.*r_com_bot.*t42.*4.0;
t209 = dtheta_1.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t42.*4.0;
t210 = dtheta_2.*dtheta_3.*l_top.*m_bot.*r_com_bot.*t42.*4.0;
t211 = -t177;
t212 = l_mid.*l_top.*m_bot.*r_com_bot.*t3.*t39;
t214 = I_bot.*I_mid.*l_top.*m_bot.*r_com_bot.*t39.*2.0;
t215 = I_bot.*l_mid.*m_mid.*t3.*t53.*2.0;
t216 = I_bot.*l_mid.*m_motor1.*t3.*t53.*2.0;
t217 = I_bot.*l_mid.*m_motor2.*t3.*t53.*2.0;
t218 = I_bot.*l_mid.*m_bot.*t3.*t60.*2.0;
t223 = l_top.*m_bot.*r_com_bot.*t14.*t42.*2.0;
t224 = l_top.*m_bot.*r_com_bot.*t15.*t42.*2.0;
t225 = l_top.*m_bot.*r_com_bot.*t16.*t42.*2.0;
t226 = l_mid.*m_bot.*t2.*t19.*t39;
t227 = m_mid.*r_com_mid.*t2.*t19.*t39;
t228 = m_motor1.*r_com_mid.*t2.*t19.*t39;
t229 = m_motor2.*r_com_mid.*t2.*t19.*t39;
t234 = m_top.*t29.*t144;
t235 = m_top.*t29.*t145;
t236 = m_top.*t29.*t146;
t241 = t35.*t195;
t242 = m_mid.*t19.*t195;
t243 = m_motor1.*t19.*t195;
t244 = m_motor2.*t19.*t195;
t246 = m_mid.*t25.*t196;
t247 = m_top.*t29.*t195;
t249 = m_motor1.*t25.*t196;
t251 = m_motor2.*t25.*t196;
t253 = m_motor1.*t25.*t141.*2.0;
t254 = m_motor2.*t25.*t141.*2.0;
t255 = m_motor2.*t25.*t142.*2.0;
t256 = I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t19.*t34.*2.0;
t257 = I_bot.*l_mid.*m_bot.*m_motor1.*r_com_mid.*t19.*t34.*2.0;
t258 = I_bot.*l_mid.*m_bot.*m_motor2.*r_com_mid.*t19.*t34.*2.0;
t263 = t34.*t197;
t265 = t34.*t200;
t266 = t35.*t202;
t267 = t34.*t203;
t268 = t34.*t204;
t276 = m_motor1.*t19.*t34.*t76.*-2.0;
t277 = m_motor2.*t19.*t34.*t76.*-2.0;
t278 = m_motor2.*t19.*t34.*t77.*-2.0;
t279 = I_bot.*m_mid.*r_com_mid.*t2.*t39.*t53.*2.0;
t280 = I_bot.*m_motor1.*r_com_mid.*t2.*t39.*t53.*2.0;
t281 = I_bot.*m_motor2.*r_com_mid.*t2.*t39.*t53.*2.0;
t291 = I_bot.*l_mid.*r_com_bot.*t2.*t19.*t20.*t39.*2.0;
t292 = I_mid.*l_mid.*l_top.*t3.*t20.*t27.*t39.*2.0;
t296 = t34.*t237;
t297 = t35.*t237;
t301 = t34.*t245;
t303 = t34.*t248;
t304 = t34.*t250;
t305 = l_mid.*m_mid.*r_com_mid.*t34.*t196.*2.0;
t306 = l_mid.*m_motor1.*r_com_mid.*t34.*t196.*2.0;
t307 = l_mid.*m_motor2.*r_com_mid.*t34.*t196.*2.0;
t319 = m_motor1.*t25.*t34.*t141.*-2.0;
t320 = m_motor2.*t25.*t34.*t141.*-2.0;
t321 = m_motor2.*t25.*t34.*t142.*-2.0;
t339 = r_com_mid.*t2.*t20.*t27.*t39.*t120.*2.0;
t340 = r_com_mid.*t2.*t20.*t27.*t39.*t121.*2.0;
t341 = r_com_mid.*t2.*t20.*t27.*t39.*t122.*2.0;
t342 = t2.*t3.*t39.*t237.*2.0;
t83 = sin(t72);
t140 = g.*m_bot.*r_com_bot.*t47;
t148 = sin(t112);
t184 = -t106;
t187 = -t107;
t188 = -t108;
t189 = -t109;
t222 = t53.*t59;
t230 = -t190;
t231 = -t192;
t232 = -t193;
t233 = -t194;
t238 = -t212;
t259 = -t226;
t260 = -t227;
t261 = -t228;
t262 = -t229;
t264 = t35.*t199;
t269 = t34.*t205;
t270 = t34.*t206;
t271 = t34.*t207;
t272 = -t256;
t273 = -t257;
t274 = -t258;
t275 = -t241;
t282 = t59.*t196;
t283 = t59.*t198;
t284 = t59.*t201;
t285 = -t263;
t287 = -t265;
t288 = -t266;
t289 = -t267;
t290 = -t268;
t293 = -t279;
t294 = -t280;
t295 = -t281;
t298 = m_mid.*t19.*t241;
t299 = m_motor1.*t19.*t241;
t300 = m_motor2.*t19.*t241;
t302 = m_top.*t29.*t241;
t308 = t34.*t253;
t309 = t34.*t254;
t310 = t34.*t255;
t311 = -t291;
t312 = -t292;
t316 = -t305;
t317 = -t306;
t318 = -t307;
t322 = t59.*t237;
t323 = -t296;
t324 = -t297;
t325 = t59.*t246;
t326 = t59.*t249;
t327 = t59.*t251;
t331 = -t301;
t333 = -t303;
t334 = -t304;
t213 = -t140;
t252 = -t222;
t286 = -t264;
t313 = -t282;
t314 = -t283;
t315 = -t284;
t328 = m_mid.*t19.*t275;
t329 = m_motor1.*t19.*t275;
t330 = m_motor2.*t19.*t275;
t332 = m_top.*t29.*t275;
t335 = -t322;
t336 = -t325;
t337 = -t326;
t338 = -t327;
t343 = t10+t131+t140+t175+t176+t223;
t344 = t9+t106+t107+t108+t109+t140+t173+t178+t180+t181+t185+t186+t211+t223;
t345 = t46+t113+t114+t115+t116+t117+t130+t132+t133+t134+t135+t136+t174+t177+t179+t182+t183+t184+t187+t188+t189+t208+t209+t210+t213+t224+t225;
t346 = t7+t52+t53+t54+t55+t56+t60+t102+t103+t104+t105+t119+t120+t121+t122+t123+t137+t138+t139+t191+t230+t231+t232+t233+t238+t252+t259+t260+t261+t262;
t347 = t36+t85+t86+t87+t88+t89+t90+t91+t92+t93+t94+t95+t96+t97+t98+t99+t100+t124+t125+t126+t127+t128+t129+t149+t150+t151+t152+t153+t154+t155+t156+t157+t158+t159+t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t197+t198+t199+t200+t201+t202+t203+t204+t205+t206+t207+t214+t215+t216+t217+t218+t219+t220+t221+t234+t235+t236+t237+t239+t240+t242+t243+t244+t245+t246+t247+t248+t249+t250+t251+t253+t254+t255+t272+t273+t274+t276+t277+t278+t285+t286+t287+t288+t289+t290+t293+t294+t295+t311+t312+t314+t315+t316+t317+t318+t319+t320+t321+t323+t324+t328+t329+t330+t331+t332+t333+t334+t335+t336+t337+t338+t339+t340+t341+t342;
t348 = 1.0./t347;
f_ss = [dtheta_1;dtheta_2;dtheta_3;((t9.*t74+t9.*t75+t9.*t76+t9.*t77+t9.*t78+t46.*t74+t46.*t75+t46.*t76+t46.*t77+t46.*t78+t9.*t118+t9.*t144+t9.*t145+t9.*t146+t76.*t113+t46.*t144+t77.*t113+t46.*t145+t74.*t117+t77.*t114+t78.*t113+t46.*t146+t75.*t117+t78.*t114+t76.*t117+t78.*t115+t77.*t117+t78.*t117-(t8.*t195)./2.0+(t9.*t195)./2.0+t76.*t130+t77.*t130+t74.*t134+t78.*t130+t74.*t135+t75.*t134+t74.*t136+t75.*t135+t75.*t136+t76.*t173+t76.*t174+t77.*t173+t77.*t174+t78.*t173+t74.*t178+t78.*t174+t74.*t179+t75.*t178+t74.*t180+t75.*t179+t74.*t181+t75.*t180+t74.*t182+t75.*t181+t74.*t183+t75.*t182+t75.*t183+t117.*t144-(t67.*t195)./2.0+t117.*t145-(t68.*t195)./2.0+t117.*t146-(t69.*t195)./2.0-(t70.*t195)./2.0+t76.*t208+t76.*t209+t77.*t208+t76.*t210+t77.*t209+t78.*t208+t77.*t210+t78.*t209+t78.*t210+t76.*t223+t76.*t224+t77.*t223+t76.*t225+t77.*t224+t78.*t223+t77.*t225+t78.*t224+t78.*t225+I_bot.*I_mid.*t46+I_bot.*I_mid.*t113+I_bot.*I_mid.*t114+I_bot.*I_mid.*t115+I_bot.*I_mid.*t116+I_bot.*I_mid.*t117+I_bot.*I_mid.*t130+I_bot.*I_mid.*t132+I_bot.*I_mid.*t133+I_bot.*I_mid.*t134+I_bot.*I_mid.*t135+I_bot.*I_mid.*t136+I_bot.*I_mid.*t174+I_bot.*I_mid.*t177+I_bot.*I_mid.*t179+I_bot.*I_mid.*t182+I_bot.*I_mid.*t183+I_bot.*I_mid.*t184+I_bot.*I_mid.*t187+I_bot.*I_mid.*t188+I_bot.*I_mid.*t189+I_bot.*I_mid.*t208+I_bot.*I_mid.*t209+I_bot.*I_mid.*t210+I_bot.*I_mid.*t213+I_bot.*I_mid.*t224+I_bot.*I_mid.*t225+(t8.*t33.*t195)./2.0-(t9.*t33.*t195)./2.0+t14.*t37.*t197+t14.*t37.*t200+t14.*t38.*t199+t15.*t38.*t199+t14.*t37.*t203+t14.*t37.*t204+t14.*t37.*t205+t14.*t37.*t206+t14.*t37.*t207+t14.*t37.*t245+t14.*t37.*t248+t14.*t37.*t250+t14.*t37.*t253+t14.*t37.*t254+t14.*t37.*t255+t14.*t111.*t198+t14.*t111.*t201+t14.*t111.*t246+t14.*t111.*t249+t14.*t111.*t251+dtheta_1.*dtheta_2.*t38.*t199.*2.0+m_bot.*r_com_bot.*t10.*t192+m_bot.*r_com_bot.*t10.*t193+m_bot.*r_com_bot.*t10.*t194-(t20.*t25.*t27.*t67)./2.0-(t20.*t25.*t27.*t68)./2.0-(t22.*t25.*t27.*t66)./2.0-(t20.*t25.*t27.*t69)./2.0-(t23.*t25.*t27.*t66)./2.0-(t24.*t25.*t27.*t66)./2.0+I_mid.*m_bot.*t10.*t27+I_mid.*m_bot.*t27.*t46+I_bot.*m_mid.*t17.*t113+I_bot.*m_mid.*t27.*t113+I_mid.*m_bot.*t27.*t117+I_mid.*m_bot.*t27.*t134+I_mid.*m_bot.*t27.*t135+I_mid.*m_bot.*t27.*t136+I_mid.*m_bot.*t27.*t179+I_mid.*m_bot.*t27.*t182+I_mid.*m_bot.*t27.*t183+I_mid.*m_bot.*t27.*t187+I_mid.*m_bot.*t27.*t188+I_mid.*m_bot.*t27.*t189+I_bot.*m_motor1.*t17.*t113+I_bot.*m_motor2.*t17.*t113+I_bot.*m_motor1.*t27.*t113+I_bot.*m_motor2.*t27.*t113+I_mid.*m_mid.*t27.*t113+I_mid.*m_motor1.*t27.*t113+I_mid.*m_motor2.*t27.*t113+I_bot.*l_mid.*m_mid.*r_com_mid.*t66+I_bot.*l_mid.*m_mid.*r_com_mid.*t208+I_bot.*l_mid.*m_mid.*r_com_mid.*t223+I_bot.*l_mid.*m_mid.*r_com_mid.*t224+I_bot.*l_mid.*m_motor1.*r_com_mid.*t66+I_bot.*l_mid.*m_motor2.*r_com_mid.*t66+I_bot.*l_mid.*m_motor1.*r_com_mid.*t208+I_bot.*l_mid.*m_motor2.*r_com_mid.*t208+I_bot.*l_mid.*m_motor1.*r_com_mid.*t223+I_bot.*l_mid.*m_motor1.*r_com_mid.*t224+I_bot.*l_mid.*m_motor2.*r_com_mid.*t223+I_bot.*l_mid.*m_motor2.*r_com_mid.*t224+(g.*l_top.*m_mid.*t62.*t195)./4.0+(g.*l_top.*m_mid.*t83.*t195)./4.0+g.*l_top.*m_motor1.*t61.*t76+g.*l_top.*m_motor2.*t61.*t76+g.*l_top.*m_motor2.*t61.*t77+g.*l_top.*m_motor1.*t61.*t144+g.*l_top.*m_motor2.*t61.*t144+g.*l_top.*m_motor2.*t61.*t145+(g.*l_top.*m_motor1.*t62.*t195)./4.0+(g.*l_top.*m_motor2.*t62.*t195)./4.0+(g.*l_top.*m_motor1.*t83.*t195)./4.0+(g.*l_top.*m_motor2.*t83.*t195)./4.0+(g.*m_top.*r_com_top.*t62.*t195)./4.0+(g.*m_top.*r_com_top.*t83.*t195)./4.0+m_mid.*m_motor1.*t25.*t27.*t113+m_mid.*m_motor2.*t25.*t27.*t113+m_motor1.*m_motor2.*t25.*t27.*t113+(l_mid.*r_com_mid.*t20.*t27.*t67)./2.0+(l_mid.*r_com_mid.*t20.*t27.*t68)./2.0+(l_mid.*r_com_mid.*t20.*t27.*t69)./2.0+l_mid.*m_mid.*r_com_mid.*t14.*t37.*t196-l_mid.*m_mid.*r_com_mid.*t14.*t111.*t196+l_mid.*m_motor1.*r_com_mid.*t14.*t37.*t196+l_mid.*m_motor2.*r_com_mid.*t14.*t37.*t196-l_mid.*m_motor1.*r_com_mid.*t14.*t111.*t196-l_mid.*m_motor2.*r_com_mid.*t14.*t111.*t196+l_top.*m_mid.*r_com_mid.*t5.*t14.*t195+l_top.*m_mid.*r_com_mid.*t5.*t15.*t195-l_top.*m_mid.*r_com_mid.*t14.*t63.*t195-l_top.*m_mid.*r_com_mid.*t15.*t63.*t195+l_top.*m_motor1.*r_com_mid.*t5.*t14.*t195+l_top.*m_motor1.*r_com_mid.*t5.*t15.*t195+l_top.*m_motor2.*r_com_mid.*t5.*t14.*t195+l_top.*m_motor2.*r_com_mid.*t5.*t15.*t195-l_top.*m_motor1.*r_com_mid.*t14.*t63.*t195-l_top.*m_motor1.*r_com_mid.*t15.*t63.*t195-l_top.*m_motor2.*r_com_mid.*t14.*t63.*t195-l_top.*m_motor2.*r_com_mid.*t15.*t63.*t195+(l_mid.*l_top.*t2.*t9.*t20.*t27)./2.0-(l_mid.*l_top.*t2.*t10.*t20.*t27)./2.0-(l_mid.*l_top.*t9.*t20.*t27.*t57)./2.0+(l_mid.*l_top.*t10.*t20.*t27.*t57)./2.0+(l_top.*r_com_bot.*t10.*t17.*t20.*t39)./2.0-(l_top.*r_com_bot.*t10.*t17.*t20.*t81)./2.0+(I_mid.*g.*l_mid.*t20.*t27.*sin(t11+t31))./2.0+I_bot.*l_mid.*l_top.*m_bot.*t2.*t9-(I_bot.*g.*l_top.*t4.*t17.*t20)./2.0-(I_bot.*g.*l_top.*t4.*t20.*t27)./2.0-(I_bot.*g.*l_top.*t4.*t22.*t25)./2.0-(I_bot.*g.*l_top.*t4.*t23.*t25)./2.0-(I_bot.*g.*l_top.*t4.*t24.*t25)./2.0+(I_bot.*g.*l_top.*t17.*t20.*t61)./2.0+(I_bot.*g.*l_top.*t22.*t25.*t61)./2.0+(I_bot.*g.*l_top.*t23.*t25.*t61)./2.0+(I_bot.*g.*l_top.*t24.*t25.*t61)./2.0+(I_bot.*g.*l_top.*t20.*t27.*t148)./2.0-(I_mid.*g.*l_mid.*t20.*t27.*t40)./2.0-(I_mid.*g.*l_top.*t4.*t20.*t27)./2.0+(I_mid.*g.*l_top.*t20.*t27.*t148)./2.0-I_bot.*l_mid.*m_bot.*r_com_bot.*t3.*t8.*2.0+I_bot.*l_top.*m_bot.*r_com_bot.*t9.*t39+I_mid.*l_mid.*m_bot.*r_com_bot.*t3.*t10+I_bot.*l_top.*m_mid.*r_com_mid.*t2.*t9+I_mid.*l_top.*m_bot.*r_com_bot.*t10.*t39+I_bot.*l_top.*m_motor1.*r_com_mid.*t2.*t9+I_bot.*l_top.*m_motor2.*r_com_mid.*t2.*t9+I_bot.*m_mid.*r_com_mid.*t14.*t53.*t64.*2.0+I_bot.*m_motor1.*r_com_mid.*t14.*t53.*t64.*2.0+I_bot.*m_motor2.*r_com_mid.*t14.*t53.*t64.*2.0+I_bot.*l_top.*t5.*t14.*t18.*t20.*2.0+I_bot.*l_top.*t5.*t15.*t18.*t20.*2.0+I_bot.*l_top.*t5.*t14.*t22.*t26.*2.0+I_bot.*l_top.*t5.*t14.*t23.*t26.*2.0+I_bot.*l_top.*t5.*t15.*t22.*t26.*2.0+I_bot.*l_top.*t5.*t14.*t24.*t26.*2.0+I_bot.*l_top.*t5.*t15.*t23.*t26.*2.0+I_bot.*l_top.*t5.*t15.*t24.*t26.*2.0+I_bot.*l_top.*t14.*t20.*t28.*t42.*2.0+I_bot.*l_top.*t15.*t20.*t28.*t42.*2.0+I_bot.*l_top.*t16.*t20.*t28.*t42.*2.0+I_mid.*l_mid.*t6.*t14.*t20.*t28.*2.0+I_mid.*l_mid.*t6.*t15.*t20.*t28.*2.0+I_mid.*l_mid.*t6.*t16.*t20.*t28.*2.0+I_mid.*l_top.*t14.*t20.*t28.*t42.*2.0+I_mid.*l_top.*t15.*t20.*t28.*t42.*2.0+I_mid.*l_top.*t16.*t20.*t28.*t42.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t18.*t20.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t22.*t26.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t23.*t26.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t5.*t24.*t26.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*t20.*t28.*t42.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*t20.*t28.*t42.*4.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*t20.*t28.*t42.*4.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*t6.*t20.*t28.*4.0+I_mid.*dtheta_1.*dtheta_3.*l_mid.*t6.*t20.*t28.*4.0+I_mid.*dtheta_2.*dtheta_3.*l_mid.*t6.*t20.*t28.*4.0+I_mid.*dtheta_1.*dtheta_2.*l_top.*t20.*t28.*t42.*4.0+I_mid.*dtheta_1.*dtheta_3.*l_top.*t20.*t28.*t42.*4.0+I_mid.*dtheta_2.*dtheta_3.*l_top.*t20.*t28.*t42.*4.0-(I_bot.*g.*l_mid.*l_top.*r_com_bot.*t20.*t41)./2.0-(I_bot.*g.*l_mid.*l_top.*r_com_bot.*t20.*t82)./2.0+I_bot.*g.*l_mid.*l_top.*r_com_bot.*t20.*t101+I_bot.*l_top.*m_mid.*m_motor1.*t5.*t14.*t26.*4.0+I_bot.*l_top.*m_mid.*m_motor1.*t5.*t15.*t26.*4.0+I_bot.*l_top.*m_mid.*m_motor2.*t5.*t14.*t26.*4.0+I_bot.*l_top.*m_mid.*m_motor2.*t5.*t15.*t26.*4.0+I_bot.*l_top.*m_motor1.*m_motor2.*t5.*t14.*t26.*4.0+I_bot.*l_top.*m_motor1.*m_motor2.*t5.*t15.*t26.*4.0+I_bot.*l_mid.*l_top.*t5.*t14.*t20.*t27.*4.0+I_bot.*l_mid.*l_top.*t5.*t15.*t20.*t27.*4.0+I_bot.*l_mid.*l_top.*t5.*t16.*t20.*t27.*3.0+I_bot.*l_mid.*l_top.*t14.*t20.*t27.*t63.*2.0+I_bot.*l_mid.*l_top.*t15.*t20.*t27.*t63.*2.0+I_bot.*l_mid.*l_top.*t16.*t20.*t27.*t63+I_mid.*l_mid.*l_top.*t5.*t15.*t20.*t27+I_mid.*l_mid.*l_top.*t14.*t20.*t27.*t63.*2.0+I_mid.*l_mid.*l_top.*t15.*t20.*t27.*t63+I_bot.*l_mid.*r_com_bot.*t14.*t19.*t20.*t64.*2.0+I_bot.*l_top.*r_com_bot.*t14.*t17.*t20.*t42.*4.0+I_bot.*l_top.*r_com_bot.*t15.*t17.*t20.*t42.*4.0+I_bot.*l_top.*r_com_bot.*t16.*t17.*t20.*t42+I_bot.*l_top.*r_com_bot.*t14.*t17.*t20.*t84.*2.0+I_bot.*l_top.*r_com_bot.*t15.*t17.*t20.*t84.*2.0+I_bot.*l_top.*r_com_bot.*t16.*t17.*t20.*t84+dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t5.*t195.*2.0-dtheta_1.*dtheta_2.*l_top.*m_mid.*r_com_mid.*t63.*t195.*2.0+dtheta_1.*dtheta_2.*l_top.*m_motor1.*r_com_mid.*t5.*t195.*2.0+dtheta_1.*dtheta_2.*l_top.*m_motor2.*r_com_mid.*t5.*t195.*2.0-dtheta_1.*dtheta_2.*l_top.*m_motor1.*r_com_mid.*t63.*t195.*2.0-dtheta_1.*dtheta_2.*l_top.*m_motor2.*r_com_mid.*t63.*t195.*2.0+(g.*l_top.*m_bot.*t22.*t25.*t27.*t61)./2.0+(g.*l_top.*m_bot.*t23.*t25.*t27.*t61)./2.0+(g.*l_top.*m_bot.*t24.*t25.*t27.*t61)./2.0+(g.*l_top.*m_mid.*t20.*t25.*t27.*t148)./2.0+(g.*l_top.*m_motor1.*t20.*t25.*t27.*t148)./2.0+(g.*l_top.*m_motor2.*t20.*t25.*t27.*t148)./2.0+l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t9.*t27-l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t10.*t27+l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t9.*t27-l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t10.*t27+l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t9.*t27-l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t10.*t27+l_top.*m_bot.*t5.*t14.*t22.*t26.*t27.*2.0+l_top.*m_bot.*t5.*t14.*t23.*t26.*t27.*2.0+l_top.*m_bot.*t5.*t15.*t22.*t26.*t27.*2.0+l_top.*m_bot.*t5.*t14.*t24.*t26.*t27.*2.0+l_top.*m_bot.*t5.*t15.*t23.*t26.*t27.*2.0+l_top.*m_bot.*t5.*t15.*t24.*t26.*t27.*2.0+l_top.*m_mid.*t14.*t20.*t25.*t28.*t42.*2.0+l_top.*m_mid.*t15.*t20.*t25.*t28.*t42.*2.0+l_top.*m_mid.*t16.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor1.*t14.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor1.*t15.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor2.*t14.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor1.*t16.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor2.*t15.*t20.*t25.*t28.*t42.*2.0+l_top.*m_motor2.*t16.*t20.*t25.*t28.*t42.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*m_mid.*m_motor1.*t5.*t26.*8.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*m_mid.*m_motor2.*t5.*t26.*8.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*m_motor1.*m_motor2.*t5.*t26.*8.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t5.*t20.*t27.*8.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*t5.*t20.*t27.*6.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*t5.*t20.*t27.*6.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t20.*t27.*t63.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*t20.*t27.*t63.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*t20.*t27.*t63.*2.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t5.*t20.*t27.*2.0+I_mid.*dtheta_1.*dtheta_2.*l_mid.*l_top.*t20.*t27.*t63.*2.0-I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_bot.*t41+I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*t61-I_bot.*g.*l_mid.*l_top.*m_bot.*m_mid.*r_com_bot.*t82-I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_bot.*t41-I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_bot.*t41+I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*t61+I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*t61-I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_bot.*t82-I_bot.*g.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_bot.*t82+I_bot.*dtheta_1.*dtheta_2.*l_top.*r_com_bot.*t17.*t20.*t42.*8.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*r_com_bot.*t17.*t20.*t42.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*r_com_bot.*t17.*t20.*t42.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_top.*r_com_bot.*t17.*t20.*t84.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_top.*r_com_bot.*t17.*t20.*t84.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_top.*r_com_bot.*t17.*t20.*t84.*2.0+(I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t41)./2.0-I_bot.*g.*l_mid.*m_bot.*m_top.*r_com_bot.*r_com_top.*t41+(I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t82)./2.0+I_bot.*g.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t101-I_bot.*g.*l_mid.*m_bot.*m_top.*r_com_bot.*r_com_top.*t82+(I_bot.*g.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t41)./2.0+(I_bot.*g.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t41)./2.0+(I_bot.*g.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t82)./2.0+(I_bot.*g.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t82)./2.0+I_bot.*g.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t101+I_bot.*g.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t101+I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t14.*t19.*t37.*2.0+I_bot.*l_mid.*m_bot.*m_motor1.*r_com_mid.*t14.*t19.*t37.*2.0+I_bot.*l_mid.*m_bot.*m_motor2.*r_com_mid.*t14.*t19.*t37.*2.0+dtheta_1.*dtheta_2.*l_top.*m_bot.*t5.*t22.*t26.*t27.*4.0+dtheta_1.*dtheta_2.*l_top.*m_bot.*t5.*t23.*t26.*t27.*4.0+dtheta_1.*dtheta_2.*l_top.*m_bot.*t5.*t24.*t26.*t27.*4.0+dtheta_1.*dtheta_2.*l_top.*m_mid.*t20.*t25.*t28.*t42.*4.0+dtheta_1.*dtheta_3.*l_top.*m_mid.*t20.*t25.*t28.*t42.*4.0+dtheta_2.*dtheta_3.*l_top.*m_mid.*t20.*t25.*t28.*t42.*4.0+dtheta_1.*dtheta_2.*l_top.*m_motor1.*t20.*t25.*t28.*t42.*4.0+dtheta_1.*dtheta_2.*l_top.*m_motor2.*t20.*t25.*t28.*t42.*4.0+dtheta_1.*dtheta_3.*l_top.*m_motor1.*t20.*t25.*t28.*t42.*4.0+dtheta_1.*dtheta_3.*l_top.*m_motor2.*t20.*t25.*t28.*t42.*4.0+dtheta_2.*dtheta_3.*l_top.*m_motor1.*t20.*t25.*t28.*t42.*4.0+dtheta_2.*dtheta_3.*l_top.*m_motor2.*t20.*t25.*t28.*t42.*4.0+(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t27.*t61)./2.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t27.*t62)./4.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t27.*t83)./4.0-(g.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t27.*t148)./2.0+(g.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t27.*t61)./2.0-(g.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t27.*t62)./4.0+(g.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t27.*t61)./2.0-(g.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t27.*t62)./4.0-(g.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t27.*t83)./4.0-(g.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t27.*t83)./4.0-(g.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t27.*t148)./2.0-(g.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t27.*t148)./2.0-(l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t10.*t39)./2.0-(l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t10.*t81)./2.0-(l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t10.*t39)./2.0-(l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t10.*t39)./2.0-(l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t10.*t81)./2.0-(l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t10.*t81)./2.0+l_top.*m_bot.*m_mid.*m_motor1.*t5.*t14.*t26.*t27.*4.0+l_top.*m_bot.*m_mid.*m_motor1.*t5.*t15.*t26.*t27.*4.0+l_top.*m_bot.*m_mid.*m_motor2.*t5.*t14.*t26.*t27.*4.0+l_top.*m_bot.*m_mid.*m_motor2.*t5.*t15.*t26.*t27.*4.0+l_top.*m_bot.*m_motor1.*m_motor2.*t5.*t14.*t26.*t27.*4.0+l_top.*m_bot.*m_motor1.*m_motor2.*t5.*t15.*t26.*t27.*4.0-l_mid.*l_top.*m_mid.*r_com_mid.*t14.*t20.*t28.*t42-l_mid.*l_top.*m_mid.*r_com_mid.*t15.*t20.*t28.*t42-l_mid.*l_top.*m_mid.*r_com_mid.*t16.*t20.*t28.*t42+l_mid.*l_top.*m_mid.*r_com_mid.*t14.*t20.*t28.*t84+l_mid.*l_top.*m_mid.*r_com_mid.*t15.*t20.*t28.*t84+l_mid.*l_top.*m_mid.*r_com_mid.*t16.*t20.*t28.*t84-l_mid.*l_top.*m_motor1.*r_com_mid.*t14.*t20.*t28.*t42-l_mid.*l_top.*m_motor1.*r_com_mid.*t15.*t20.*t28.*t42-l_mid.*l_top.*m_motor2.*r_com_mid.*t14.*t20.*t28.*t42-l_mid.*l_top.*m_motor1.*r_com_mid.*t16.*t20.*t28.*t42-l_mid.*l_top.*m_motor2.*r_com_mid.*t15.*t20.*t28.*t42-l_mid.*l_top.*m_motor2.*r_com_mid.*t16.*t20.*t28.*t42+l_mid.*l_top.*m_motor1.*r_com_mid.*t14.*t20.*t28.*t84+l_mid.*l_top.*m_motor1.*r_com_mid.*t15.*t20.*t28.*t84+l_mid.*l_top.*m_motor2.*r_com_mid.*t14.*t20.*t28.*t84+l_mid.*l_top.*m_motor1.*r_com_mid.*t16.*t20.*t28.*t84+l_mid.*l_top.*m_motor2.*r_com_mid.*t15.*t20.*t28.*t84+l_mid.*l_top.*m_motor2.*r_com_mid.*t16.*t20.*t28.*t84+l_mid.*l_top.*m_mid.*t5.*t14.*t20.*t25.*t27+l_mid.*l_top.*m_mid.*t5.*t15.*t20.*t25.*t27+l_mid.*l_top.*m_mid.*t14.*t20.*t25.*t27.*t63+l_mid.*l_top.*m_mid.*t15.*t20.*t25.*t27.*t63+l_mid.*l_top.*m_motor1.*t5.*t14.*t20.*t25.*t27+l_mid.*l_top.*m_motor1.*t5.*t15.*t20.*t25.*t27+l_mid.*l_top.*m_motor2.*t5.*t14.*t20.*t25.*t27+l_mid.*l_top.*m_motor2.*t5.*t15.*t20.*t25.*t27+l_mid.*l_top.*m_motor1.*t14.*t20.*t25.*t27.*t63+l_mid.*l_top.*m_motor1.*t15.*t20.*t25.*t27.*t63+l_mid.*l_top.*m_motor2.*t14.*t20.*t25.*t27.*t63+l_mid.*l_top.*m_motor2.*t15.*t20.*t25.*t27.*t63-I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t16.*t42+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t14.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t15.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t16.*t84-I_bot.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t16.*t42-I_bot.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t16.*t42+I_bot.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t14.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t15.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t14.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t16.*t84+I_bot.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t15.*t84.*2.0+I_bot.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t16.*t84+dtheta_1.*dtheta_2.*l_top.*m_bot.*m_mid.*m_motor1.*t5.*t26.*t27.*8.0+dtheta_1.*dtheta_2.*l_top.*m_bot.*m_mid.*m_motor2.*t5.*t26.*t27.*8.0+dtheta_1.*dtheta_2.*l_top.*m_bot.*m_motor1.*m_motor2.*t5.*t26.*t27.*8.0-dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_1.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_2.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t42.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_1.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_2.*dtheta_3.*l_mid.*l_top.*m_mid.*r_com_mid.*t20.*t28.*t84.*2.0-dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_1.*dtheta_3.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_1.*dtheta_3.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_2.*dtheta_3.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t42.*2.0-dtheta_2.*dtheta_3.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t42.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_1.*dtheta_3.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_1.*dtheta_3.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_2.*dtheta_3.*l_mid.*l_top.*m_motor1.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_2.*dtheta_3.*l_mid.*l_top.*m_motor2.*r_com_mid.*t20.*t28.*t84.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*t5.*t20.*t25.*t27.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_mid.*t20.*t25.*t27.*t63.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor1.*t5.*t20.*t25.*t27.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor2.*t5.*t20.*t25.*t27.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor1.*t20.*t25.*t27.*t63.*2.0+dtheta_1.*dtheta_2.*l_mid.*l_top.*m_motor2.*t20.*t25.*t27.*t63.*2.0-I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t42.*2.0-I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t42.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t84.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t84.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*r_com_bot.*t84.*2.0-I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t42.*2.0-I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t42.*2.0-I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t42.*2.0-I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t42.*2.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t84.*4.0+I_bot.*dtheta_1.*dtheta_2.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t84.*4.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t84.*2.0+I_bot.*dtheta_1.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t84.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*r_com_bot.*t84.*2.0+I_bot.*dtheta_2.*dtheta_3.*l_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*r_com_bot.*t84.*2.0).*2.0)./(t36.*2.0+t85.*2.0+t86.*2.0+t87.*2.0+t88.*2.0+t89.*2.0+t90.*2.0+t91.*2.0+t92.*2.0+t93.*2.0+t94.*2.0+t95.*2.0+t96.*2.0+t97.*2.0+t98.*2.0+t99.*2.0+t100.*2.0+t149.*2.0+t150.*2.0+t151.*2.0+t152.*2.0+t153.*2.0+t154.*2.0+t155.*2.0+t156.*2.0+t157.*2.0+t158.*2.0+t159.*2.0+t160.*2.0+t161.*2.0+t162.*2.0+t163.*2.0+t164.*2.0+t165.*2.0+t166.*2.0+t167.*2.0+t168.*2.0+t169.*2.0+t170.*2.0+t171.*2.0+t172.*2.0+t197+t198+t199+t200+t201+t202+t203+t204+t205+t206+t207+t234.*2.0+t235.*2.0+t236.*2.0+t239+t240+t242+t243+t244+t245+t246+t247+t248+t249+t250+t251+t253+t254+t255-t32.*t197-t32.*t200-t33.*t199-t32.*t203-t33.*t202-t32.*t204-t33.*t242-t33.*t243-t32.*t245-t33.*t244-t32.*t248-t33.*t247-t32.*t250-t110.*t198-t110.*t201-t110.*t246-t110.*t249-t110.*t251-l_mid.*m_mid.*r_com_mid.*t196-l_mid.*m_motor1.*r_com_mid.*t196-l_mid.*m_motor2.*r_com_mid.*t196-m_motor1.*t19.*t32.*t76.*2.0-m_motor2.*t19.*t32.*t76.*2.0-m_motor2.*t19.*t32.*t77.*2.0-m_motor1.*t25.*t32.*t141.*2.0-m_motor2.*t25.*t32.*t141.*2.0-m_motor2.*t25.*t32.*t142.*2.0+I_bot.*l_mid.*m_bot.*t3.*t7.*4.0+I_bot.*l_mid.*m_bot.*t3.*t60.*4.0+I_bot.*l_mid.*m_mid.*t3.*t53.*4.0+I_bot.*l_mid.*m_motor1.*t3.*t53.*4.0+I_bot.*l_mid.*m_motor2.*t3.*t53.*4.0-I_bot.*m_mid.*r_com_mid.*t3.*t53.*2.0-I_bot.*m_mid.*r_com_mid.*t53.*t58.*2.0-I_bot.*m_motor1.*r_com_mid.*t3.*t53.*2.0-I_bot.*m_motor2.*r_com_mid.*t3.*t53.*2.0-I_bot.*m_motor1.*r_com_mid.*t53.*t58.*2.0-I_bot.*m_motor2.*r_com_mid.*t53.*t58.*2.0-l_mid.*m_mid.*r_com_mid.*t32.*t196+l_mid.*m_mid.*r_com_mid.*t33.*t196+l_mid.*m_mid.*r_com_mid.*t110.*t196-l_mid.*m_motor1.*r_com_mid.*t32.*t196+l_mid.*m_motor1.*r_com_mid.*t33.*t196-l_mid.*m_motor2.*r_com_mid.*t32.*t196+l_mid.*m_motor2.*r_com_mid.*t33.*t196+l_mid.*m_motor1.*r_com_mid.*t110.*t196+l_mid.*m_motor2.*r_com_mid.*t110.*t196+I_bot.*I_mid.*l_mid.*l_top.*m_bot.*t2.*4.0+I_bot.*I_mid.*l_mid.*m_bot.*r_com_bot.*t3.*4.0+I_bot.*I_mid.*l_top.*m_bot.*r_com_bot.*t39.*4.0+I_bot.*I_mid.*l_top.*m_mid.*r_com_mid.*t2.*4.0+I_bot.*I_mid.*l_top.*m_motor1.*r_com_mid.*t2.*4.0+I_bot.*I_mid.*l_top.*m_motor2.*r_com_mid.*t2.*4.0-I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t19.*2.0-I_bot.*l_mid.*m_bot.*m_motor1.*r_com_mid.*t19.*2.0-I_bot.*l_mid.*m_bot.*m_motor2.*r_com_mid.*t19.*2.0-I_mid.*l_mid.*l_top.*t20.*t27.*t57.*2.0-I_bot.*l_mid.*r_com_bot.*t19.*t20.*t58.*2.0-I_bot.*l_mid.*m_bot.*m_mid.*r_com_mid.*t19.*t32.*2.0-I_bot.*l_mid.*m_bot.*m_motor1.*r_com_mid.*t19.*t32.*2.0-I_bot.*l_mid.*m_bot.*m_motor2.*r_com_mid.*t19.*t32.*2.0+I_mid.*l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t27.*4.0+I_mid.*l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t27.*4.0+I_mid.*l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t27.*4.0);-t344.*t348.*(t74+t75+t76+t77+t78+t79+t118+t141+t142+t143+t144+t145+t146+t147+t195+t196+t275+t313+I_bot.*I_top+I_bot.*m_bot.*t19+I_bot.*m_mid.*t19+I_bot.*m_motor1.*t19+I_bot.*m_motor2.*t19+I_bot.*m_top.*t29+I_bot.*l_mid.*l_top.*m_bot.*t2.*2.0+I_bot.*l_top.*m_bot.*r_com_bot.*t39.*2.0+I_bot.*l_top.*m_mid.*r_com_mid.*t2.*2.0+I_bot.*l_top.*m_motor1.*r_com_mid.*t2.*2.0+I_bot.*l_top.*m_motor2.*r_com_mid.*t2.*2.0+l_mid.*l_top.*t2.*t20.*t27.*2.0+l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t27.*2.0+l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t27.*2.0+l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t27.*2.0-l_mid.*l_top.*t3.*t20.*t27.*t39.*2.0)-t345.*t348.*(t74+t75+t76+t77+t78+t118+t144+t145+t146+t195+t275+I_bot.*l_mid.*l_top.*m_bot.*t2+I_bot.*l_top.*m_bot.*r_com_bot.*t39+I_bot.*l_top.*m_mid.*r_com_mid.*t2+I_bot.*l_top.*m_motor1.*r_com_mid.*t2+I_bot.*l_top.*m_motor2.*r_com_mid.*t2+l_mid.*l_top.*t2.*t20.*t27+l_top.*m_bot.*m_mid.*r_com_mid.*t2.*t27+l_top.*m_bot.*m_motor1.*r_com_mid.*t2.*t27+l_top.*m_bot.*m_motor2.*r_com_mid.*t2.*t27-l_mid.*l_top.*t3.*t20.*t27.*t39)+m_bot.*r_com_bot.*t343.*t346.*t348;-t343.*t348.*(t79+t141+t142+t143+t147+t196+t313+I_mid.*I_top+I_mid.*m_bot.*t17+I_mid.*m_bot.*t19+I_mid.*m_bot.*t27+I_top.*m_bot.*t17+I_mid.*m_mid.*t19+I_mid.*m_mid.*t25+I_mid.*m_motor1.*t19+I_mid.*m_motor2.*t19+I_mid.*m_motor1.*t25+I_mid.*m_motor2.*t25+I_top.*m_mid.*t25+I_mid.*m_top.*t29+I_top.*m_motor1.*t25+I_top.*m_motor2.*t25+t17.*t19.*t20+t19.*t22.*t25+t19.*t23.*t25+t19.*t24.*t25+l_mid.*m_bot.*t3.*t7.*2.0+l_mid.*m_bot.*t3.*t60.*2.0+l_mid.*m_mid.*t3.*t53.*2.0+l_mid.*m_motor1.*t3.*t53.*2.0+l_mid.*m_motor2.*t3.*t53.*2.0+m_bot.*m_mid.*t17.*t19+m_bot.*m_mid.*t19.*t25+m_bot.*m_motor1.*t17.*t19+m_bot.*m_motor2.*t17.*t19+m_bot.*m_motor1.*t19.*t25+m_bot.*m_motor2.*t19.*t25+m_bot.*m_top.*t17.*t29+m_mid.*m_motor1.*t19.*t25.*2.0+m_mid.*m_motor2.*t19.*t25.*2.0+m_motor1.*m_motor2.*t19.*t25.*2.0+m_mid.*m_top.*t25.*t29+m_motor1.*m_top.*t25.*t29+m_motor2.*m_top.*t25.*t29-t17.*t19.*t20.*t34-t19.*t22.*t25.*t34-t19.*t23.*t25.*t34-t19.*t24.*t25.*t34+I_mid.*l_mid.*l_top.*m_bot.*t2.*2.0+I_mid.*l_mid.*m_bot.*r_com_bot.*t3.*2.0+I_mid.*l_top.*m_bot.*r_com_bot.*t39.*2.0+I_mid.*l_top.*m_mid.*r_com_mid.*t2.*2.0+I_mid.*l_top.*m_motor1.*r_com_mid.*t2.*2.0+I_mid.*l_top.*m_motor2.*r_com_mid.*t2.*2.0-m_mid.*m_motor1.*t19.*t25.*t34.*2.0-m_mid.*m_motor2.*t19.*t25.*t34.*2.0-m_motor1.*m_motor2.*t19.*t25.*t34.*2.0+l_mid.*r_com_bot.*t3.*t19.*t20.*2.0-m_mid.*r_com_mid.*t2.*t39.*t53.*2.0-m_motor1.*r_com_mid.*t2.*t39.*t53.*2.0-m_motor2.*r_com_mid.*t2.*t39.*t53.*2.0-l_mid.*m_bot.*m_mid.*r_com_mid.*t19.*t34.*2.0-l_mid.*m_bot.*m_motor1.*r_com_mid.*t19.*t34.*2.0-l_mid.*m_bot.*m_motor2.*r_com_mid.*t19.*t34.*2.0-l_mid.*r_com_bot.*t2.*t19.*t20.*t39.*2.0)+m_bot.*r_com_bot.*t345.*t348.*(t102+t103+t104+t105+t137+t138+t139+t191+t230+t231+t232+t233+t238-I_mid.*r_com_bot-I_mid.*l_mid.*t3-I_mid.*l_top.*t39)+m_bot.*r_com_bot.*t344.*t346.*t348];
