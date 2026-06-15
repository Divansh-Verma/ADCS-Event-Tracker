function [omega_measured, q_measured] = read_sensors(omega_true, q_true)
    % read_sensors adds realistic hardware noise to the true physical state.
    
    % --- 1. Rate Gyroscope Model ---
    % Standard deviation of gyro noise (e.g., 0.01 deg/s converted to rad/s)
    gyro_noise_std = (0.01 * pi / 180); 
    
    % Add zero-mean Gaussian noise to the true angular velocity
    omega_measured = omega_true + gyro_noise_std * randn(3,1);
    
    % --- 2. Star Tracker Model ---
    % Standard deviation of attitude measurement noise
    tracker_noise_std = 1e-3; 
    
    % Add noise to the quaternion components
    q_measured = q_true + tracker_noise_std * randn(4,1);
    
    % A quaternion must always represent a pure rotation, so we re-normalize it
    q_measured = q_measured / norm(q_measured); 
end