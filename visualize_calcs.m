clear;
close all

% change these angles to configure arm placement
arm = Arm([pi/2,pi/3]);

g_uCOM = Arm.get_config(arm.origin_shoulder()*arm.shoulder_uCOM());
g_fCOM = Arm.get_config(arm.origin_shoulder()*arm.shoulder_elbow()*arm.elbow_fCOM());
Tq = arm.get_Tq();
a_u = atan2(g_uCOM(2),g_uCOM(1));
a_f = atan2(g_fCOM(2),g_fCOM(1));

planarR2_display(arm.theta, [arm.l1, arm.l2]); hold on

s = 0.07; % scale torque dimension
plot(g_uCOM(1),g_uCOM(2),'*r')
plot(g_fCOM(1),g_fCOM(2),'*r')
quiver(g_uCOM(1), g_uCOM(2), s*Tq(1)*sin(a_u), s*-Tq(1)*cos(a_u));
quiver(g_fCOM(1), g_fCOM(2), s*Tq(2)*sin(a_f), s*-Tq(2)*cos(a_f));
title(['Net Force: ' num2str(sum(arm.get_Fg())) 'N, Net Torque: ' num2str(sum(arm.get_Tq())) 'Nm'])
