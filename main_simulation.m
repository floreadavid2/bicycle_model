% MAIN_SIMULATION Runs the baseline linear 2-DOF Bicycle Model simulation
clear; clc; close all;

% 1. Load vehicle parameters and matrices
p = vehicle_params();

fprintf('Understeer gradient (eta): %.4f rad (%.2f deg/g)\n', p.eta, rad2deg(p.eta));
fprintf('Critical velocity (u_CR):  %.2f m/s\n', p.u_crit);

% 2. Simulation setup
tspan = [0 10];                   % Simulation time [s]
x0    = [0; 0];                   % Initial states: [v0; r0]
delta_func = @(t) (t >= 1) * 0.1; % Step steering input: 0.1 rad at t = 1s

% 3. Numerical integration (ODE45)
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', 0.01);
[t_out, x_out] = ode45(@(t, x) bicycle_ode(t, x, delta_func, p), tspan, x0, options);

% 4. Post-processing using State-Space output equation: y = C*x + D*delta
delta_vec = delta_func(t_out');               % Row vector of steering inputs (1 x N)
y_out     = (p.C * x_out' + p.D * delta_vec)'; % Output matrix (N x 4)

r_out    = y_out(:, 1); % Yaw rate [rad/s]
v_out    = y_out(:, 2); % Lateral velocity [m/s]
ay_out   = y_out(:, 3); % Lateral acceleration [m/s^2]
beta_out = y_out(:, 4); % Vehicle sideslip angle [rad]

% 5. Plotting results
figure('Name', 'Baseline 2-DOF Bicycle Model - Step Steer Response', 'Color', 'w');

subplot(2, 2, 1);
plot(t_out, rad2deg(r_out), 'LineWidth', 1.5);
title('Yaw Rate (r)');
xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
grid on;

subplot(2, 2, 2);
plot(t_out, v_out, 'LineWidth', 1.5);
title('Lateral Velocity (v)');
xlabel('Time [s]'); ylabel('Velocity [m/s]');
grid on;

subplot(2, 2, 3);
plot(t_out, ay_out, 'LineWidth', 1.5);
title('Lateral Acceleration (a_y)');
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
grid on;

subplot(2, 2, 4);
plot(t_out, rad2deg(beta_out), 'LineWidth', 1.5);
title('Sideslip Angle (\beta)');
xlabel('Time [s]'); ylabel('Angle [deg]');
grid on;