% Set mechanical bound parameters related to vehicle kinematics
global vehicle_kinematics_
vehicle_kinematics_.vehicle_v_max = 2.5;
vehicle_kinematics_.vehicle_a_max = 1.0;
vehicle_kinematics_.vehicle_phy_max = 0.7;
vehicle_kinematics_.vehicle_w_max = 0.5;
vehicle_kinematics_.threshold_s = (vehicle_kinematics_.vehicle_v_max^2) / vehicle_kinematics_.vehicle_a_max;

% Set geometric parameters related to vehicle
global vehicle_geometrics_
vehicle_geometrics_.vehicle_wheelbase = 2.8;
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;

vehicle_kinematics_.vehicle_kappa_max = tan(vehicle_kinematics_.vehicle_phy_max) / vehicle_geometrics_.vehicle_wheelbase;
vehicle_kinematics_.vehicle_turning_radius_min = 1 / vehicle_kinematics_.vehicle_kappa_max;

% Set scale of environment
global environment_scale_
environment_scale_.environment_x_min = -20;
environment_scale_.environment_x_max = 20;
environment_scale_.environment_y_min = -20;
environment_scale_.environment_y_max = 20;
environment_scale_.x_scale = environment_scale_.environment_x_max - environment_scale_.environment_x_min;
environment_scale_.y_scale = environment_scale_.environment_y_max - environment_scale_.environment_y_min;

% Specify the polygonal obstacles (in Case1.mat, the obstacles are randomly generated)
global Nobs
Nobs = 5;

global hybrid_astar_
hybrid_astar_.resolution_x = 0.2;
hybrid_astar_.resolution_y = 0.2;
hybrid_astar_.resolution_theta = 0.2;

hybrid_astar_.num_nodes_x = ceil(environment_scale_.x_scale / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_y = ceil(environment_scale_.y_scale / hybrid_astar_.resolution_y) + 1;
hybrid_astar_.num_nodes_theta = ceil(2 * pi / hybrid_astar_.resolution_theta) + 1;
hybrid_astar_.penalty_for_backward = 1;
hybrid_astar_.penalty_for_direction_changes = 5;
hybrid_astar_.penalty_for_steering_changes = 0;
hybrid_astar_.multiplier_H = 3.0;
hybrid_astar_.multiplier_H_for_A_star = 3.0;
hybrid_astar_.max_iter = 500;
hybrid_astar_.simulation_step = 0.7;

global costmap_
costmap_ = CreateDilatedCostmap();

global optimization_
optimization_.Nfe = 100;
optimization_.dt_for_resampling = 0.01;
optimization_.unit_step = 0.03;
optimization_.max_step = 10;