close all
clear;

%% Plot distance of spring stretch across full range of motion
% theta is respect to the horizontal
syms theta real
% linkage links in inches, distance returned is in inches
l1 = 11;
l2 = 3;
d(theta) = sqrt(l1^2+l2^2 - 2*l1*l2*cosd(90-theta));

tt = -75:75; % max angle from -75 to 75
figure;
plot(tt,d(tt));
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Distance (in)')
title('Spring Distance with respect to 4-bar linkage angle')

%% Determine Spring Constant Parameters
F_test = [34.5/16; 34.5/16+1.25; 7.5; 15]; % in pounds
delta_x = [5/16; 1; 2+5/16; 5+5/8]; % in inches
x = [delta_x,  ones(length(delta_x),1)];
k = x\F_test % estimated spring constant of individual spring

figure;
hold on
plot(delta_x, F_test, '*r');
plot(delta_x, x*k, 'b')
xlabel('Displacement (in)')
ylabel('Force (lbs)')
title('Spring System Force Plot');

%% Supporting Magnitude Spring Force 
% equivalent spring constant of 3 springs in series as used in OmoSling
k_eq = k;
% k_eq(1) = 17; %7*k(1);
% assumption: we can choose the inital distance of the spring before
% stretching since it is dependent on the motor wind up. For these
% calculations, we will assume that the initial distance occurs when the
% 4-bar linkage is in its most up-right position (theta=75)
delta_x = d(tt)-d(75);
x = [delta_x',  ones(length(delta_x),1)];
F = x*k_eq;
F = F';
figure;
plot(tt, F)
ylim([0,max(double(F))])
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Force (lbs)')
title('Magnitude of Spring System Force')

%% Find Torque at each position
gamma = acosd((l1^2+d(tt).^2-l2^2)./(2*l1*d(tt)));
r = l1;
T = r*F.*sind(gamma);
figure;
% nexttile;
% plot(tt, gamma)
% xlabel('theta')
% ylabel('Gamma')
% title('Gamma Plot')
% nexttile;
% plot(tt, sind(gamma))
% xlabel('theta')
% ylabel('sin(Gamma)')
% title('sin(Gamma) Plot')
% nexttile;
plot(tt, T)
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Torque (lbs-in)')
title('Torque Provided by Spring System')

%% Find support Upward Force
Fy = T/l1./sind(90-tt);
figure;
plot(tt, Fy) 
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Force (lbs)')
title('Vertical Support Provided at Arm Trough')