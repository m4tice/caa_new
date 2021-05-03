# variables
img_width = 320
img_height = 160
public_sensor_tick = 0.05


# Only for town02
starts_node_indices_town02 = [2066, 1766, 355, 649]

destination_node_points_town02 = [[44, 107.4], [90, 107.4], [140, 107.4], [-5.5, 147.5], [191.7, 147.5], [20, 189.5],
                                  [90, 189.5], [165, 189.5], [-5.5, 215], [43.9, 215], [134.1, 215], [191.7, 215],
                                  [-5.5, 269], [43.9, 269], [191.7, 269], [20, 304.5], [90, 304.5], [165, 304.5]]

course_01 = [starts_node_indices_town02[0], destination_node_points_town02[11]]
course_02 = [starts_node_indices_town02[1], destination_node_points_town02[2]]
course_03 = [starts_node_indices_town02[2], destination_node_points_town02[16]]
course_04 = [starts_node_indices_town02[3], destination_node_points_town02[14]]


# Colors
orange = '245, 85, 1'
petronas_color = '1, 128, 118'
redbull_color = '18, 20, 45'


# Maps
town01 = {
    'name': 'Town01',
    'dir_name': 'town01',
    'x': 200,
    'y': 157,
    'z': 330,
    'pitch': -90,
    'yaw': 90,
    'weather': 'CloudyNoon'
}

town02 = {
    'name': 'Town02',
    'dir_name': 'town02',
    'x': 90,
    'y': 200,
    'z': 220,
    'pitch': -90,
    'yaw': 0,
    'weather': 'CloudyNoon'
}

town03 = {
    'name': 'Town03',
    'dir_name': 'town03',
    'x': 35,
    'y': 0,
    'z': 400,
    'pitch': -90,
    'yaw': 180,
    'weather': 'CloudyNoon'
}

town04 = {
    'name': 'Town04',
    'dir_name': 'town04',
    'x': -50,
    'y': 40,
    'z': 580,
    'pitch': -90,
    'yaw': 55,
    'weather': 'Default'
}

town05 = {
    'name': 'Town05',
    'dir_name': 'town05',
    'x': 0,
    'y': -10,
    'z': 420,
    'pitch': -90,
    'yaw': 90,
    'weather': 'CloudyNoon'
}

town06 = {
    'name': 'Town06',
    'dir_name': 'town06',
    'x': 150,
    'y': 120,
    'z': 550,
    'pitch': -90,
    'yaw': 90,
    'weather': 'Default'
}

town07 = {
    'name': 'Town07',
    'dir_name': 'town07',
    'x': -70,
    'y': -20,
    'z': 300,
    'pitch': -90,
    'yaw': 0,
    'weather': 'CloudyNoon'
}

town10 = {
    'name': 'Town10HD',
    'dir_name': 'town10',
    'x': -20,
    'y': 30,
    'z': 220,
    'pitch': -90,
    'yaw': 90,
    'weather': 'CloudyNoon'
}

# Weathers
weather = {
    'weathers': ['Default', 'ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon',
                 'MidRainyNoon', 'HardRainNoon', 'SoftRainNoon', 'ClearSunset', 'CloudySunset',
                 'WetSunset', 'WetCloudySunset', 'MidRainSunset', 'HardRainSunset', 'SoftRainSunset']
}

# Sensors
cam_rgb_set_1 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': '%d' % img_width,
    'image_size_y': '%d' % img_height,
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 1.2,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 1.7,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -10.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_rgb_set_2 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': '%d' % img_width,
    'image_size_y': '%d' % img_height,
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 2.3,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 0.8,  # 1.7 - 1.2 - 0.7
    'rot_pitch': 0.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_rgb_set_3 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': '%d' % img_width,
    'image_size_y': '%d' % img_height,
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 2.0,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 1.2,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -20.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_rgb_set_4 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': '%d' % img_width,
    'image_size_y': '%d' % img_height,
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 0.0,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 2.0,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -90.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_spectate_1 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': str(480),
    'image_size_y': str(240),
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': -7.0,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 5.0,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -10.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_spectate_2 = {
    'name': 'sensor.camera.rgb',
    # 'image_size_x': str(480),
    # 'image_size_y': str(240),
    # 'fov': '110',
    # 'sensor_tick': str(public_sensor_tick),
    'loc_x': -5.5,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 2.8,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -15.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_spectate_3 = {
    'name': 'sensor.camera.rgb',
    # 'image_size_x': str(480),
    # 'image_size_y': str(240),
    # 'fov': '110',
    # 'sensor_tick': str(public_sensor_tick),
    'loc_x': 0.3,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 1.2,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -5.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_spectate_4 = {
    'name': 'sensor.camera.rgb',
    'image_size_x': str(480),
    'image_size_y': str(240),
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 2.4,  # 1.2 - 2.0 - 2.5
    'loc_y': 0.0,
    'loc_z': 5.0,  # 1.7 - 1.2 - 0.7
    'rot_pitch': -90.0,  # -10.0 - -20.0 - 0.0
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

cam_ss = {
    'name': 'sensor.camera.semantic_segmentation',
    'image_size_x': '%d' % img_width,
    'image_size_y': '%d' % img_height,
    'fov': '110',
    'sensor_tick': str(public_sensor_tick),
    'loc_x': 2.5,  # 1.6 - 2.5
    'loc_y': 0.0,
    'loc_z': 0.7,  # 1.7 - 0.7
    'rot_pitch': 0.0,
    'rot_yaw': 0.0,
    'rot_roll': 0.0
}

lidar = {
    'name': 'sensor.lidar.ray_cast',
    'channels': str(64),
    'points_per_second': str(56000),
    'rotation_frequency': str(10),
    'range': str(50),
    'upper_fov': str(10),
    'lower_fov': str(-30),
    'sensor_tick': str(0)
}

lidar_ss = {
    'name': 'sensor.lidar.ray_cast_semantic',
    'channels': str(32),
    'points_per_second': str(100000),
    'rotation_frequency': str(10),
    'range': str(50),
    'upper_fov': str(10),
    'lower_fov': str(-30),
    'sensor_tick': str(0)
}

gnss = {
    'name': 'sensor.other.gnss',
    'noise_alt_bias': str(0.0), 	# Mean parameter in the noise model for altitude.
    'noise_alt_stddev': str(0.0), 	# Standard deviation parameter in the noise model for altitude.
    'noise_lat_bias': str(0.0), 	# Mean parameter in the noise model for latitude.
    'noise_lat_stddev': str(0.0), 	# Standard deviation parameter in the noise model for latitude.
    'noise_lon_bias': str(0.0), 	# Mean parameter in the noise model for longitude.
    'noise_lon_stddev': str(0.0), 	# Standard deviation parameter in the noise model for longitude.
    'noise_seed': str(0), 	        # Initializer for a pseudorandom number generator.
    'sensor_tick': str(0.05),
    'loc_x': 1.0,
    'loc_z': 2.8
}

imu = {
    'name': 'sensor.other.imu',
    'noise_accel_stddev_x': str(0.0), 	# Standard deviation parameter in the noise model for acceleration (X axis).
    'noise_accel_stddev_y': str(0.0), 	# Standard deviation parameter in the noise model for acceleration (Y axis).
    'noise_accel_stddev_z': str(0.0), 	# Standard deviation parameter in the noise model for acceleration (Z axis).
    'noise_gyro_bias_x': str(0.0), 	    # Mean parameter in the noise model for the gyroscope (X axis).
    'noise_gyro_bias_y': str(0.0), 	    # Mean parameter in the noise model for the gyroscope (Y axis).
    'noise_gyro_bias_z': str(0.0), 	    # Mean parameter in the noise model for the gyroscope (Z axis).
    'noise_gyro_stddev_x': str(0.0), 	# Standard deviation parameter in the noise model for the gyroscope (X axis).
    'noise_gyro_stddev_y': str(0.0), 	# Standard deviation parameter in the noise model for the gyroscope (Y axis).
    'noise_gyro_stddev_z': str(0.0), 	# Standard deviation parameter in the noise model for the gyroscope (Z axis).
    'noise_seed': str(0), 	            # Initializer for a pseudorandom number generator.
    'sensor_tick': str(public_sensor_tick) 	        # Simulation seconds between sensor captures (ticks).
}
