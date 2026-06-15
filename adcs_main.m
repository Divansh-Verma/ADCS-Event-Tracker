% FINAL ADCS Closed-Loop Simulation with EKF
clc; clear; close all;

% Load the Onboard Star Catalog
load('onboard_catalog.mat'); 

% --- Step 1: Define Parameters ---
inertia = [0.015, 0.020, 0.005];
dt = 0.1; % Control loop update rate (10 Hz)
t_end = 60; % Simulate for 60 seconds
num_steps = t_end / dt;

% Controller Gains
Kp = 0.01; 
Kd = 0.05; 

% Target Attitude (Identity quaternion: perfectly level)
q_target = [1; 0; 0; 0]; 

% --- Step 2: Initialize State Arrays ---
t_history = zeros(num_steps, 1);
x_true_history = zeros(num_steps, 7);
x_est_history = zeros(num_steps, 7); 

% Initial tumbling state (Fast, chaotic spin)
x_current = [0.5; -0.3; 0.2; 1; 0; 0; 0];

% EKF Initialization (Start with a perfect guess and low uncertainty)
x_est = x_current; 
P_cov = eye(7) * 0.1; 

% --- Step 3: The Closed-Loop Simulation ---
disp('Running Closed-Loop ADCS with EKF...');
for i = 1:num_steps
    t_current = (i-1) * dt;
    
    % 1. Sensors: Read the corrupted hardware data
    [omega_meas, q_meas] = read_sensors(x_current(1:3), x_current(4:7));
    z_meas = [omega_meas; q_meas];
    
    % 2. Filter: Clean the noise using the EKF
    if i == 1; tau_cmd = [0;0;0]; end 
    [x_est, P_cov] = ekf_update(x_est, P_cov, tau_cmd, z_meas, dt, inertia);
    
    % 3. Brain: Calculate torque based on the CLEAN ESTIMATE
    tau_cmd = attitude_controller(x_est(4:7), x_est(1:3), q_target, Kp, Kd);
    
    % 4. Plant: Simulate True Physics
    t_span = [t_current, t_current + dt];
    [t_out, x_out] = ode45(@(t, x) spacecraft_dynamics(t, x, inertia, tau_cmd), t_span, x_current);
    x_current = x_out(end, :)';
    
    % 5. Log Data for Dashboard
    t_history(i) = t_current;
    x_true_history(i, :) = x_current';
    x_est_history(i, :) = x_est';
end
disp('Simulation Complete! Generating Dashboard...');

% --- Step 4: Visualize EKF and Controller Performance ---
figure('Name', 'ADCS Performance Dashboard', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% --- Plot 1: Angular Velocity (Detumbling) ---
subplot(2,1,1);
plot(t_history, x_true_history(:, 1:3), 'LineWidth', 2); hold on;
set(gca, 'ColorOrderIndex', 1); % Match dashed line colors to solid line colors
plot(t_history, x_est_history(:, 1:3), '--', 'LineWidth', 1.5); 
title('Spacecraft Detumbling (Angular Velocity \rightarrow 0)');
ylabel('Angular Rate (rad/s)');
legend('\omega_x (True)', '\omega_y (True)', '\omega_z (True)', ...
       '\omega_x (EKF)', '\omega_y (EKF)', '\omega_z (EKF)');
grid on;

% --- Plot 2: Attitude Quaternions (Targeting Orientation) ---
subplot(2,1,2);
plot(t_history, x_true_history(:, 4:7), 'LineWidth', 2); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(t_history, x_est_history(:, 4:7), '--', 'LineWidth', 1.5);
title('Attitude Targeting (Quaternions \rightarrow [1, 0, 0, 0])');
xlabel('Time (seconds)');
ylabel('Quaternion Components');
legend('q_0 (True)', 'q_1 (True)', 'q_2 (True)', 'q_3 (True)', ...
       'Location', 'best');
grid on;