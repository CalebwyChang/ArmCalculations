close all
clear;

%% Plot distance of spring stretch across full range of motion
% theta is respect to the horizontal
syms theta real
% linkage links in inches, distance returned is in inches
l1 = 11;
l2 = 3;
d(theta) = sqrt(l1^2+l2^2-2*l1*l2*cosd(90-theta));

tt = -75:75; % max angle from -75 to 75
figure;
plot(tt,d(tt));
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Distance (in)')
title('Spring Distance with respect to 4-bar linkage angle')

%% Determine Spring Constant Parameter
% 2.25 inches resting
% 3.00 inches 7.5 lb weight
% 4.00 inches 15 lb weight
x1 = 2.25;
x2 = [x1, 3, 4];
F = [0, 7.5, 15]*0.453592; % convert pounds to kilograms (kg)
delta_x = (x2-x1)*0.0254; % convert inches to meters (m)
k = F/delta_x; % estimated spring constant of individual spring
figure;
hold on
plot(delta_x, F, '*r');
plot(delta_x, k.*delta_x, 'b')
xlabel('Displacement (m)')
ylabel('Force (N)')
title(['Spring Force of Individual Spring, k=' num2str(k)]);

%% Supporting Magnitude Spring Force 
% equivalent spring constant of 3 springs in series as used in OmoSling
k_eq = k/3; % 1(1/k+1/k+1/k)
% assumption: we can choose the inital distance of the spring before
% stretching since it is dependent on the motor wind up. For these
% calculations, we will assume that the initial distance occurs when the
% 4-bar linkage is in its most up-right position (theta=75)
x1 = d(75); 
delta_x = abs(d(tt)-x1);
delta_x = delta_x*0.0254; % convert to m
figure;
yyaxis left
plot(tt, delta_x.*k_eq)
ylim([0,max(double(delta_x.*k_eq))])
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Force (N)')
title('Magnitude of Spring Force')
grid on
yyaxis right
plot(tt, delta_x.*k_eq*0.2248)
ylim([0,max(double(delta_x.*k_eq*0.2248))])
ylabel('Pound-Force (lbs)')

%% Find Torque at each position
gamma = acosd((l1^2+d(tt).^2-l2^3)./(2*l1*d(tt)));
r = l1;
F = delta_x.*k_eq;
T = r*F.*sind(gamma);
figure;
plot(tt, T)
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Torque (Nm)')
title('Torque Provided by Spring')

%% Find support Upward Force
l3 = 9;
alpha = 180-tt;
r_support = sqrt(l1^2+l3^2-2*l1*l3*cosd(alpha));
zeta = asind(l1*sind(alpha)./r_support);
delta = 90-zeta;
r_support = r_support*0.0254; % convert to meters
F_support = T./r_support;
Fy = F_support./sind(delta);
figure;
plot(tt, Fy*0.2248) % convert to pound-force
xticks(-75:15:75)
xlabel('Angle (degrees)')
ylabel('Pound-Force (lbs)')
title('Vertical Support Provided at Arm Trough')