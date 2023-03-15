clear;
close all

% change these angles to configure arm placement
theta = [2*pi/3,0];
len = [0.38, 0.25];
p_COM = [0.57, 0.45];
mass = [3.25, 1.87+0.65];
arm = Arm(theta, len, p_COM, mass);
support_distance = 0.0762; % 3 inches from elbow (converted to meters)

arm.show(support_distance);