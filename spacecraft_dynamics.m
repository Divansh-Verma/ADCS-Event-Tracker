function x_dot = spacecraft_dynamics(t, x, inertia, tau)
    % x_dot = spacecraft_dynamics(t, x, inertia, tau)
    %
    % State vector x:
    % x(1:3) = omega_x, omega_y, omega_z (angular velocity in rad/s)
    % x(4:7) = q0, q1, q2, q3           (quaternion orientation)
    %
    % Inputs:
    % inertia = [I_xx, I_yy, I_zz]      (principal moments of inertia in kg*m^2)
    % tau = [tau_x, tau_y, tau_z]       (applied control torques in N*m)

    % Unpack inertia
    I_xx = inertia(1);
    I_yy = inertia(2);
    I_zz = inertia(3);

    % Unpack torques
    tau_x = tau(1);
    tau_y = tau(2);
    tau_z = tau(3);

    % Unpack states
    omega = x(1:3);
    q = x(4:7);
    
    % Ensure quaternion remains normalized (good numerical practice)
    q = q / norm(q);

    % 1. Kinetics (Euler's equations of motion)
    omega_dot = zeros(3,1);
    omega_dot(1) = (tau_x - (I_zz - I_yy) * omega(2) * omega(3)) / I_xx;
    omega_dot(2) = (tau_y - (I_xx - I_zz) * omega(1) * omega(3)) / I_yy;
    omega_dot(3) = (tau_z - (I_yy - I_xx) * omega(1) * omega(2)) / I_zz;

    % 2. Kinematics (Quaternion derivative)
    % Construct the skew-symmetric matrix Omega
    Omega = [  0, -omega(1), -omega(2), -omega(3);
             omega(1),   0,  omega(3), -omega(2);
             omega(2), -omega(3),   0,  omega(1);
             omega(3),  omega(2), -omega(1),   0 ];
         
    q_dot = 0.5 * Omega * q;

    % Assemble the full 7x1 state derivative vector
    x_dot = [omega_dot; q_dot];
end