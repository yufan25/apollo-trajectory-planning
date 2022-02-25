#pragma once

#define FLAGS_max_s_lateral_optimization 60.0
#define FLAGS_default_delta_s_lateral_optimization 1.0
#define FLAGS_lateral_optimization 0
#define FLAGS_default_reference_line_width 4.0
#define FLAGS_bound_buffer 0.1
#define FLAGS_longitudinal_acceleration_lower_bound -4.5
#define FLAGS_longitudinal_acceleration_upper_bound 4
#define FLAGS_lateral_third_order_derivative_max 0.1
#define FLAGS_weight_lateral_obstacle_distance 0.0
#define FLAGS_enable_osqp_debug false
#define FLAGS_weight_lateral_offset 1.0
#define FLAGS_weight_lateral_obstacle_distance 0.0
#define FLAGS_weight_lateral_derivative 500.0
#define FLAGS_weight_lateral_second_order_derivative 1000.0
#define FLAGS_trajectory_time_length 8.0  //planning的时间长度
#define FLAGS_polynomial_minimal_param 0.01
#define FLAGS_num_velocity_sample 6
#define FLAGS_min_velocity_sample_gap 1.0
#define FLAGS_numerical_epsilon 1e-6
#define FLAGS_time_min_density 1.0
#define FLAGS_default_lon_buffer 5.0
#define FLAGS_speed_lon_decision_horizon 200.0 //纵向规划的距离，200m长了点吧？？
#define FLAGS_num_sample_follow_per_timestamp 3

//trajectory.evaluator.cpp
#define FLAGS_trajectory_time_resolution 0.1
#define FLAGS_lattice_stop_buffer 0.02
#define FLAGS_trajectory_space_resolution 1.0

#define FLAGS_weight_lon_objective 10.0
#define FLAGS_weight_lon_jerk 1.0
#define FLAGS_weight_lon_collision 5.0
#define FLAGS_weight_centripetal_acceleration 1.5
#define FLAGS_weight_lat_offset 3.0
#define FLAGS_weight_lat_comfort 3.0
#define FLAGS_weight_opposite_side_offset 10.0
#define FLAGS_weight_same_side_offset 1.0
#define FLAGS_weight_target_speed 1.0
#define FLAGS_weight_dist_travelled 10.0

#define FLAGS_lat_offset_bound 3.0
#define FLAGS_longitudinal_jerk_upper_bound 2.0
#define FLAGS_longitudinal_jerk_lower_bound -4.0
#define FLAGS_lon_collision_cost_std 0.5
#define FLAGS_lon_collision_yield_buffer 1.0
#define FLAGS_lon_collision_overtake_buffer 5.0
#define FLAGS_comfort_acceleration_factor 0.5

//constraint_checker1d.cpp
#define FLAGS_speed_lower_bound -0.1
#define FLAGS_speed_upper_bound 40.0
#define FLAGS_lateral_acceleration_bound 4.0
#define FLAGS_lateral_jerk_bound 4.0

//collision_checker.cc
#define FLAGS_lon_collision_buffer 2.0
#define FLAGS_lat_collision_buffer 0.1

//constraint_checker.cpp
#define FLAGS_kappa_bound 0.1979
// #define FLAGS_kappa_bound 30.1

#define FLAGS_use_navigation_mode 0