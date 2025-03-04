% Clear workspace and close figures
clear; close all; clc;

% Define robot using DH parameters (2-DOF planar)
L1 = Link('d', 0, 'a', 1, 'alpha', 0); % Link 1 (length = 1)
L2 = Link('d', 0, 'a', 1, 'alpha', 0); % Link 2 (length = 1)
robot = SerialLink([L1 L2], 'name', '2-DOF Robot');

% Desired end-effector velocity (world y-axis)
vy = 0.1; % m/s (constant velocity along y)

% Simulation parameters
dt = 0.05; % Time step (s)
T = 4; % Total simulation time (s)
t_steps = 0:dt:T;

% Initial joint angles (radians)
q_current = [pi/6, 4*pi/6]; % Avoid singularity at [0,0]

% Initialize plot
figure;
robot.plot(q_current, 'nobase', 'noshadow', 'noname');
hold on;
title('End-Effector Moving at Constant Velocity in Y');
axis([-2 2 -2 2]);
view(0,90)
grid on;

% Pre-allocate for logging (optional)
y_positions = zeros(1, length(t_steps));

J = robot.jacob0(q_current);
J_xy = J(1:2, :);

% Simulation loop
for i = 1:length(t_steps)
    % Compute current Jacobian (first 2 rows for x-y linear velocity)
    J = robot.jacob0(q_current);
    J_xy = J(1:2, :); % Extract x-y components
    
    % Check for singularity (determinant near 0)
    if abs(det(J_xy)) < 1e-3
        disp('Singularity! Stopping simulation.');
        break;
    end
    
    % Desired end-effector velocity [vx; vy]
    v_desired = [0; vy];
    
    % Solve for joint velocities: q_dot = J_xy \ v_desired
    q_dot = J_xy \ v_desired;
    
    % Update joint angles (Euler integration)
    q_current = q_current + q_dot' * dt;
    
    % Optional: Log end-effector y-position
    T_ee = robot.fkine(q_current);
    y_positions(i) = T_ee.t(2);
    
    % Animate
    robot.plot(q_current, 'nobase', 'noshadow', 'noname', 'delay', 0);
    pause(dt);
end

% Plot the y-position vs time
figure;
plot(t_steps(1:i-1), y_positions(1:i-1));
xlabel('Time (s)');
ylabel('End-Effector Y-Position (m)');
title('Y-Position Over Time');
grid on;