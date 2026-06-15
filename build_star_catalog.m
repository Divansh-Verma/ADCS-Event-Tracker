% build_star_catalog.m
% Run this script ONCE to generate the onboard star catalog from hip_main.csv.

clc; clear; close all;

% --- 1. Load the Raw Data ---
disp('Loading hip_main.csv... (This might take a few seconds)');
% readtable handles CSV headers automatically
raw_data = readtable('hip_main.csv', 'VariableNamingRule', 'preserve');

% Extract the relevant columns
HIP_ID  = raw_data.HIP_Number;
RA_deg  = raw_data.RA_Deg;
Dec_deg = raw_data.Dec_Deg;
V_mag   = raw_data.Vmag;

% --- 2. Filter by Brightness (Visual Magnitude) ---
% Real trackers can only see stars brighter than a certain magnitude
mag_threshold = 5.5; 

% Find indices of valid, bright stars (excluding missing NaN values)
valid_idx = (V_mag <= mag_threshold) & ~isnan(V_mag) & ~isnan(RA_deg) & ~isnan(Dec_deg);

RA_filtered  = RA_deg(valid_idx);
Dec_filtered = Dec_deg(valid_idx);
HIP_filtered = HIP_ID(valid_idx);

fprintf('Processed full catalog: Filtered from %d down to %d stars.\n', height(raw_data), length(RA_filtered));

% --- 3. Convert to 3D Inertial Unit Vectors ---
% Convert degrees to radians for trigonometric functions
RA_rad  = deg2rad(RA_filtered);
Dec_rad = deg2rad(Dec_filtered);

% Pre-allocate the catalog matrix (N x 3)
num_stars = length(RA_rad);
star_catalog_inertial = zeros(num_stars, 3);

for i = 1:num_stars
    % Standard spherical to Cartesian conversion
    v_x = cos(Dec_rad(i)) * cos(RA_rad(i));
    v_y = cos(Dec_rad(i)) * sin(RA_rad(i));
    v_z = sin(Dec_rad(i));
    
    star_catalog_inertial(i, :) = [v_x, v_y, v_z];
end

% --- 4. Save to a lightweight .mat file ---
% We also save the HIP_ID just in case you want to print the star names later!
save('onboard_catalog.mat', 'star_catalog_inertial', 'HIP_filtered');
disp('Success: onboard_catalog.mat created and ready for simulation.');