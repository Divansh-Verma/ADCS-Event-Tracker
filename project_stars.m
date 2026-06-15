function [u_px, v_px, visible_star_ids] = project_stars(q_true, star_catalog, star_ids)
    % project_stars simulates a pinhole camera capturing a starfield.
    
    % --- 1. Camera Hardware Specifications ---
    % Assume a standard aerospace star tracker camera
    FOV_deg = 20; % Field of View in degrees
    resolution = 1024; % 1024x1024 pixel sensor
    pixel_pitch_mm = 0.015; % Size of one pixel (15 micrometers)
    
    % Calculate focal length (f) based on FOV and sensor size
    sensor_size_mm = resolution * pixel_pitch_mm;
    f_mm = (sensor_size_mm / 2) / tand(FOV_deg / 2);
    
    % --- 2. Convert Quaternion to Rotation Matrix (DCM) ---
    % Assuming scalar-first quaternion: q = [q0, q1, q2, q3]
    q0 = q_true(1); q1 = q_true(2); q2 = q_true(3); q3 = q_true(4);
    
    A = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2 + q0*q3),   2*(q1*q3 - q0*q2);
         2*(q1*q2 - q0*q3),   q0^2-q1^2+q2^2-q3^2, 2*(q2*q3 + q0*q1);
         2*(q1*q3 + q0*q2),   2*(q2*q3 - q0*q1),   q0^2-q1^2-q2^2+q3^2];
     
    % --- 3. Transform Stars to Camera Body Frame ---
    % star_catalog is N x 3. We transpose to multiply, then transpose back.
    % v_body will be N x 3: [X, Y, Z]
    v_body = (A * star_catalog')'; 
    
    % --- 4. Filter Stars (Keep only those in front of the camera, Z > 0) ---
    front_idx = v_body(:, 3) > 0;
    v_front = v_body(front_idx, :);
    ids_front = star_ids(front_idx);
    
    if isempty(v_front)
        u_px = []; v_px = []; visible_star_ids = [];
        return;
    end
    
    % --- 5. Pinhole Projection (3D to 2D) ---
    % x_mm = f * (X/Z)
    x_mm = f_mm * (v_front(:, 1) ./ v_front(:, 3));
    y_mm = f_mm * (v_front(:, 2) ./ v_front(:, 3));
    
    % Convert to pixel coordinates (centered at resolution/2)
    u_raw = (x_mm / pixel_pitch_mm) + (resolution / 2);
    v_raw = (y_mm / pixel_pitch_mm) + (resolution / 2);
    
    % --- 6. Filter by Sensor Boundaries (Keep only stars inside the FOV) ---
    on_sensor_idx = (u_raw >= 1) & (u_raw <= resolution) & ...
                    (v_raw >= 1) & (v_raw <= resolution);
                
    u_px = u_raw(on_sensor_idx);
    v_px = v_raw(on_sensor_idx);
    visible_star_ids = ids_front(on_sensor_idx);
end