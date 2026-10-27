-- Standalone real-lidar mapping: no robot model or sensor extrinsics required.
include "smartcar_2d_real.lua"
options.tracking_frame = "laser_link"
options.published_frame = "laser_link"
options.provide_odom_frame = false
options.use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
return options
