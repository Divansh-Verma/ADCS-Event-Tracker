function tau_cmd = attitude_controller(q_measured, omega_measured, q_target, Kp, Kd)
    % attitude_controller calculates the control torque using a PD law.
    
    % Ensure quaternions are normalized
    q_m = q_measured / norm(q_measured);
    q_t = q_target / norm(q_target);
    
    % Calculate the inverse (conjugate) of the target quaternion
    % Assuming scalar-first convention: [q0, q1, q2, q3]
    q_t_inv = [q_t(1); -q_t(2:4)]; 
    
    % Calculate Error Quaternion: q_e = q_target_inverse * q_measured
    % Quaternion multiplication matrix for q_t_inv
    Q_matrix = [ q_t_inv(1), -q_t_inv(2), -q_t_inv(3), -q_t_inv(4);
                 q_t_inv(2),  q_t_inv(1), -q_t_inv(4),  q_t_inv(3);
                 q_t_inv(3),  q_t_inv(4),  q_t_inv(1), -q_t_inv(2);
                 q_t_inv(4), -q_t_inv(3),  q_t_inv(2),  q_t_inv(1) ];
             
    q_error = Q_matrix * q_m;
    
    % Extract the vector part of the error quaternion
    q_err_vector = q_error(2:4);
    
    % PD Control Law
    % If the scalar part q_error(1) is negative, we take the shortest path
    if q_error(1) < 0
        tau_cmd = Kp * q_err_vector - Kd * omega_measured;
    else
        tau_cmd = -Kp * q_err_vector - Kd * omega_measured;
    end
    
    % Actuator Saturation (Reaction wheels have maximum torque limits)
    max_torque = 0.005; % 5 mNm max torque
    tau_cmd = max(min(tau_cmd, max_torque), -max_torque);
end