% Initialising
clear;close all;clc;

%% System Modelling
a1 = 0.3/12*8;
a2 = 0.3;
L(1) = Link('d', 0, 'a', a1, 'alpha', 0, 'standard'); % Link 1 (length = a1)
L(2) = Link('d', 0, 'a', a2, 'alpha', 0, 'standard'); % Link 2 (length = a2)
Two_Link = SerialLink([L(1),L(2)]);

%% Obtain Jacobian matrix from forward dynamics
syms theta1 theta2
x = -a1*sin(theta1) - a2*sin(theta1 + theta2); % zero ref!!
y = a1*cos(theta1) + a2*cos(theta1 + theta2);
X = [x;y];
J = jacobian(X,[theta1 theta2]);

%% Obtain the initial joint velocity by initial endeffecor velocity (zero)
%J_w = [1 1];
%J = [J_v;J_w]
dX = [0;0];
q_current = [pi/6, 4*pi/6];
dtheta = vpa(subs(inv(J),[theta1 theta2],[q_current(1)-pi/2 q_current(2)])*dX);

%% Make the animation
dt = 0.05;
T = 6;
title('2-DOF Robot Animation (Velocity Control)');
axis([-2 2 -2 2]);
t_steps = 0:dt:T;
dtheta_m = zeros(1, length(t_steps));
view(0,90)
grid on;
i = 1;
for t = 0:dt:T
   dX = [0;0.1*sin(t)];
   q_current = q_current + double(dtheta.')*dt;
   dtheta_m(i) = dtheta(2);
   dtheta= vpa(subs(inv(J),[theta1 theta2],[q_current(1)-pi/2 q_current(2)])*dX);
   Two_Link.plot(q_current, 'nobase', 'noshadow', 'noname', 'delay', 0);
   i = i + 1;
   pause(dt)
end

% Plot the y-position vs time
figure
plot(0:dt:T,dtheta_m)
xlabel('Time (s)');
ylabel('End-Effector Y-Velocity (m/s)');
title('Y-Velocity Over Time');
grid on;