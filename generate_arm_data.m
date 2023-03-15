theta1s = (0:10:100)*pi/180;
theta2s = (90:-10:0)*pi/180;
theta2s = [theta2s 0];
support_distance = 0.0762; % 3 inches from elbow (converted to meters)

% https://andreaportman.tripod.com/averages.html
% https://pubmed.ncbi.nlm.nih.gov/2562693/#:~:text=In%20all%20the%20examined%20material,mean%2D23.30)%20of%20length.
len = [0.38, 0.25]; 
p_COM = [0.57, 0.45]; % http://www.kdm.p.lodz.pl/articles/2017/3/21_3_4.pdf
mass = [3.25, 1.87+0.65]; % https://exrx.net/Kinesiology/Segments


% output of data
out = zeros(2,length(theta1s));

for i=1:length(theta1s)
    theta = [theta1s(i), theta2s(i)];
    arm = Arm(theta, len, p_COM, mass);
    F_support = arm.get_support(support_distance); 
    out(:,i) = F_support(1:2);
    figure;
    arm.show(support_distance); % comment this line out if no figures are wanted
end