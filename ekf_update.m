function [x_est, P_new] = ekf_update(x_prev, P_prev, u_cmd, z_meas, dt, inertia)
    % ekf_update runs one cycle of the Extended Kalman Filter
    
    % --- 1. Filter Tuning Matrices ---
    % Q: Process Noise (How much we distrust our physics model)
    Q = eye(7) * 1e-6; 
    
    % R: Measurement Noise (How much we distrust our sensors)
    % Gyros are decent, but our star tracker is very noisy
    R = diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-2]); 
    
    % H: Observation Matrix (Sensors directly measure the 7 states)
    H = eye(7);
    
    % --- 2. PREDICT STEP ---
    % Predict next state using a simple Euler integration of your physics
    x_dot = spacecraft_dynamics(0, x_prev, inertia, u_cmd);
    x_pred = x_prev + x_dot * dt;
    x_pred(4:7) = x_pred(4:7) / norm(x_pred(4:7)); % Normalize quaternion
    
    % Compute Jacobian F numerically (Perturbation method)
    F = eye(7);
    delta = 1e-5;
    for j = 1:7
        x_pert = x_prev;
        x_pert(j) = x_pert(j) + delta;
        x_dot_pert = spacecraft_dynamics(0, x_pert, inertia, u_cmd);
        x_pred_pert = x_pert + x_dot_pert * dt;
        
        % The column of F is the rate of change of the state prediction
        F(:, j) = (x_pred_pert - x_pred) / delta;
    end
    
    % Predict Covariance
    P_pred = F * P_prev * F' + Q;
    
    % --- 3. UPDATE STEP ---
    % Measurement Residual (Difference between sensors and prediction)
    y = z_meas - H * x_pred;
    
    % Kalman Gain
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    % Update State Estimate
    x_est = x_pred + K * y;
    x_est(4:7) = x_est(4:7) / norm(x_est(4:7)); % Re-normalize quaternion
    
    % Update Covariance
    P_new = (eye(7) - K * H) * P_pred;
end