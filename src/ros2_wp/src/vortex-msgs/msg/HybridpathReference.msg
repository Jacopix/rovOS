# Message for Hybrid reference path signal.

float64 w # w = s_dot - v_s
float64 v_s # Desired speed assignment
float64 v_ss # Derivative of desired speed assignment
geometry_msgs/Pose2D eta_d # Desired position
geometry_msgs/Pose2D eta_d_s # Derivative of desired position
geometry_msgs/Pose2D eta_d_ss # Second derivative of desired position
