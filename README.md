# ADCS-Event-Tracker
# Software-in-the-Loop ADCS & Event-Based Star Tracker Simulator

**Author:** Divansh Verma  
**Language:** MATLAB  

##  Project Overview
This project is a complete Software-in-the-Loop (SIL) simulation of a Spacecraft Attitude Determination and Control System (ADCS). It features a non-linear 6-DOF rigid body physics engine, a custom Extended Kalman Filter (EKF) for state estimation, and a closed-loop PD controller. 

Uniquely, this simulation includes a **Digital Twin of a Neuromorphic Event Camera**. It maps real celestial data from the Hipparcos catalog, dynamically projects it through a simulated pinhole camera lens, and generates an asynchronous neuromorphic event stream ($+1$/$-1$ polarity) based on sub-pixel threshold crossings during spacecraft tumbling maneuvers.

![ADCS Performance Dashboard](docs/adcs_dashboard.png)
*(Above: The closed-loop controller successfully detumbling the spacecraft from 0.5 rad/s to 0 rad/s while the EKF accurately tracks the true state through heavy sensor noise.)*

##  System Architecture

1. **The Plant (`spacecraft_dynamics.m`):** Integrates Euler's rotational equations of motion and quaternion kinematics to simulate rigid-body dynamics in a vacuum.
2. **The Sensors (`project_stars.m` & `generate_events.m`):** - Simulates Gaussian noise and random walk for rate gyroscopes.
   - Converts 3D inertial star vectors into a 2D camera body frame.
   - Generates asynchronous pixel-level events based on temporal brightness changes.
3. **The Filter (`ekf_update.m`):** An Extended Kalman Filter utilizing a numerically computed Jacobian to fuse noisy rate data and absolute attitude measurements into a clean state estimate.
4. **The Controller (`attitude_controller.m`):** A quaternion-based Proportional-Derivative (PD) feedback loop calculating the restorative torques required from the reaction wheels.

##  Quick Start / How to Run

1. Clone this repository.
2. Ensure you have the Aerospace Blockset / Optimization toolboxes (if required by your MATLAB version, though core code relies on standard `ode45`).
3. **First Run:** Execute `src/build_star_catalog.m` to parse the Hipparcos dataset and generate the lightweight `onboard_catalog.mat` memory map.
4. **Run Simulation:** Execute `src/adcs_main.m`. This will run the 60-second closed-loop simulation and automatically generate the performance dashboard plot.
