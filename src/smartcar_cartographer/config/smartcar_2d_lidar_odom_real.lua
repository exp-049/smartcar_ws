-- Real chassis, lidar-only odometry.
--
-- The left wheel exposes a single encoder phase, so its direction is inferred
-- from the commanded target and the resulting wheel twist swings by several
-- rad/s on a straight command. Feeding that into the estimator corrupts both
-- the map and the Nav2 local costmap, so the scan matcher supplies motion here
-- instead. IMU stays off for the same reason: without trustworthy wheel motion
-- the gravity filter has nothing to subtract and would inject a constant tilt.
--
-- Frame ownership is unchanged from smartcar_2d.lua: Cartographer still emits
-- map -> chassis_odom -> base_link, which Nav2 and the costmaps require.
include "smartcar_2d.lua"
-- The live sensor HAL publishes its empirically remapped IMU vectors in
-- base_link, so Cartographer's IMU tracking frame must be colocated there.
options.tracking_frame = "base_link"
options.use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Real-lidar scans are noisier and sparser than the simulated sensor, so the
-- matcher needs more correspondence candidates per scan and a looser insert
-- threshold; the defaults reject nearly every submap insertion at rest.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
return options
