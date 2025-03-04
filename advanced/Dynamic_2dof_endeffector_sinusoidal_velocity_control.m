% Clear workspace and close figures
clear;close all;clc;


syms theta1 theta2

a1 = 0.3/12*8;
a2 = 0.3;
L(1) = Link('d', 0, 'a', a1, 'alpha', 0, 'standard'); % Link 1 (length = a1)
L(2) = Link('d', 0, 'a', a2, 'alpha', 0, 'standard'); % Link 2 (length = a2)
Two_Link = SerialLink([L(1),L(2)]);
%q_current = [0.00000049999583334722219742066247793327;0.025]
%Two_Link.plot(q_current.')
%Two_Link.teach

%%
%syms a1 a2
x = -a1*sin(theta1) - a2*sin(theta1 + theta2); % zero ref!!
y = a1*cos(theta1) + a2*cos(theta1 + theta2);
X = [x;y]
J = jacobian(X,[theta1 theta2])
%dJ = jacobian(J,[theta1 theta2])
%%
J_w = [1 1];
%J = [J_v;J_w]
dX = [0;0];
q_current = [pi/6, 4*pi/6];
dtheta = vpa(subs(inv(J),[theta1 theta2],[q_current(1)-pi/2 q_current(2)])*dX);

dt = 0.05;
T = 6;
title('2-DOF Robot Animation (Velocity Control)');
%% 
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
figure(2)
plot(dtheta_m)