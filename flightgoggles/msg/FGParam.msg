# ROS Message that contains parameters for running flightgoggles.

Header header

# sim attributes (x = width y = height)
string                sim_scene_name
geometry_msgs/Point32 sim_cam_dimensions
float32               sim_cam_fov

# levels of evaluation attributes [0 - 1]
float32               level_illumination
float32               level_features
float32               level_clutter
float32               level_motion_blur

# illumination changes direction (up, down, random)
string                illumination_changes_direction

# camera distortion parameter
# describes the applied fisheye distortion s
float32               cam_fisheye_distortion
bool                  cam_is_grayscale

# noise and timedelays
float32               cam_time_delay
float32               imu_acc_noise
float32               imu_gyro_noise
float32               imu_acc_randomwalk_noise
float32               imu_gyro_randomwalk_noise

# camera extrinsics
geometry_msgs/TransformStamped cam_imu_transform
