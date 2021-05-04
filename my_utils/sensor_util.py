import glob
import os
import sys

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

except IndexError:
    pass

import carla

import math
import csv
import cv2
import time
import random
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(r"../../")
from caa.c_utils import training_util as tu

# -- DEFINE VARIABLES --
IM_WIDTH = 320
IM_HEIGHT = 160


# == SPAWN FUNCTIONS =========
# -- Create camera RGB -
def spawn_camera_rgb(world, blueprint_library, attached_object, dic_set):
    cam_rgb_bp = blueprint_library.find(dic_set['name'])

    if 'image_size_x' in dic_set:
        cam_rgb_bp.set_attribute('image_size_x', dic_set['image_size_x'])

    if 'image_size_y' in dic_set:
        cam_rgb_bp.set_attribute('image_size_y', dic_set['image_size_y'])

    if 'fov' in dic_set:
        cam_rgb_bp.set_attribute('fov', dic_set['fov'])

    if 'sensor_tick' in dic_set:
        cam_rgb_bp.set_attribute('sensor_tick', dic_set['sensor_tick'])

    cam_rgb_sp = carla.Transform(carla.Location(x=dic_set['loc_x'], z=dic_set['loc_z']),
                                 carla.Rotation(pitch=dic_set['rot_pitch']))
    cam_rgb = world.spawn_actor(cam_rgb_bp, cam_rgb_sp, attach_to=attached_object)

    return cam_rgb


# -- Create camera RGB -
def spawn_camera_ss(world, blueprint_library, attached_object, dic_set):
    cam_ss_bp = blueprint_library.find(dic_set['name'])
    cam_ss_bp.set_attribute('image_size_x', dic_set['image_size_x'])
    cam_ss_bp.set_attribute('image_size_y', dic_set['image_size_y'])
    cam_ss_bp.set_attribute('fov', dic_set['fov'])
    cam_ss_bp.set_attribute('sensor_tick', dic_set['sensor_tick'])
    cam_ss_sp = carla.Transform(carla.Location(x=dic_set['loc_x'], z=dic_set['loc_z']),
                                carla.Rotation(pitch=dic_set['rot_pitch']))
    cam_ss = world.spawn_actor(cam_ss_bp, cam_ss_sp, attach_to=attached_object)

    return cam_ss


def spawn_lidar(world, blueprint_library, attached_object, dic_set):
    lidar_bp = blueprint_library.find(dic_set['name'])
    lidar_bp.set_attribute('channels', dic_set['channels'])
    lidar_bp.set_attribute('points_per_second', dic_set['points_per_second'])
    lidar_bp.set_attribute('rotation_frequency', dic_set['rotation_frequency'])
    lidar_bp.set_attribute('range', dic_set['range'])
    lidar_bp.set_attribute('upper_fov', dic_set['upper_fov'])
    lidar_bp.set_attribute('lower_fov', dic_set['lower_fov'])
    lidar_location = carla.Location(0, 0, 2)
    lidar_rotation = carla.Rotation(0, 0, 0)
    lidar_transform = carla.Transform(lidar_location, lidar_rotation)
    lidar_sen = world.spawn_actor(lidar_bp, lidar_transform, attach_to=attached_object)

    return lidar_sen


def spawn_gnss(world, blueprint_library, attached_object, dic_set):
    gnss_bp = blueprint_library.find(dic_set['name'])
    gnss_bp.set_attribute('noise_alt_bias', dic_set['noise_alt_bias'])
    gnss_bp.set_attribute('noise_alt_stddev', dic_set['noise_alt_stddev'])
    gnss_bp.set_attribute('noise_lat_bias', dic_set['noise_lat_bias'])
    gnss_bp.set_attribute('noise_lat_stddev', dic_set['noise_lat_stddev'])
    gnss_bp.set_attribute('noise_lon_bias', dic_set['noise_lon_bias'])
    gnss_bp.set_attribute('noise_lon_stddev', dic_set['noise_lon_stddev'])
    gnss_bp.set_attribute('noise_seed', dic_set['noise_seed'])
    gnss_bp.set_attribute('sensor_tick', dic_set['sensor_tick'])
    gnss_transform = carla.Transform(carla.Location(x=dic_set['loc_x'], z=dic_set['loc_z']))
    gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=attached_object)

    return gnss


def spawn_imu(world, blueprint_library, attached_object, dic_set):
    imu_bp = blueprint_library.find(dic_set['name'])
    imu_bp.set_attribute('noise_accel_stddev_x', dic_set['noise_accel_stddev_x'])
    imu_bp.set_attribute('noise_accel_stddev_y', dic_set['noise_accel_stddev_y'])
    imu_bp.set_attribute('noise_accel_stddev_z', dic_set['noise_accel_stddev_z'])
    imu_bp.set_attribute('noise_gyro_bias_x', dic_set['noise_gyro_bias_x'])
    imu_bp.set_attribute('noise_gyro_bias_y', dic_set['noise_gyro_bias_y'])
    imu_bp.set_attribute('noise_gyro_bias_z', dic_set['noise_gyro_bias_z'])
    imu_bp.set_attribute('noise_gyro_stddev_x', dic_set['noise_gyro_stddev_x'])
    imu_bp.set_attribute('noise_gyro_stddev_y', dic_set['noise_gyro_stddev_y'])
    imu_bp.set_attribute('noise_gyro_stddev_z', dic_set['noise_gyro_stddev_z'])
    imu_bp.set_attribute('noise_seed', dic_set['noise_seed'])
    imu_bp.set_attribute('sensor_tick', dic_set['sensor_tick'])
    imu_transform = carla.Transform()

    imu = world.spawn_actor(imu_bp, imu_transform, attach_to=attached_object)

    return imu


def spawn_vehicle(world, blueprint_library, model, color=None, spawn_number=None):
    car_bp = blueprint_library.filter(model)[0]

    # set color
    if color is not None:
        car_bp.set_attribute('color', color)

    if spawn_number is None:
        car_sp = random.choice(world.get_map().get_spawn_points())
    else:
        car_sps = world.get_map().get_spawn_points()
        car_sp = car_sps[spawn_number]

    vehicle = world.spawn_actor(car_bp, car_sp)

    return vehicle


def spawn_mpc_vehicle(world, blueprint_library, initial_state, model, color=None,):
    car_bp = blueprint_library.filter(model)[0]

    # set color
    if color is not None:
        car_bp.set_attribute('color', color)

    car_sp = carla.Transform(carla.Location(x=initial_state.x, y=initial_state.y, z=0.500000),
                             carla.Rotation(pitch=0.000000, yaw=initial_state.yaw, roll=0.000000))

    vehicle = world.spawn_actor(car_bp, car_sp)

    return vehicle


# == CAMERA FUNCTIONS =====
# -- Display image -----
def live_cam(image, dic_set, pp1=False):
    cam_height, cam_width = int(dic_set['image_size_y']), int(dic_set['image_size_x'])
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (cam_height, cam_width, 4))
    image = array[:, :, :3]

    if pp1:
        image = image[:, :, ::-1]
        image = tu.preprocess1(image)

    cv2.imshow("back camera", image)
    cv2.waitKey(1)


def camera_drive_function(data, control_unit, world, weathers, dynamic_weather=False, min_spd=2, max_spd=4):
    control_unit.auto_switch_02(data, min_spd, max_spd, camera=False)
    # control_unit.level_one(data, min_spd, max_spd)

    if dynamic_weather:
        num = random.randint(0, 1000)
        if num == 350 or num == 650:
            weather_type = random.choice(weathers)
            print("Weather changed to:", weather_type)
            world.set_weather(getattr(carla.WeatherParameters, weather_type))  # set weather


# -- MOST IMPORTANT::Sensor based: Record data -------
def record_data(image, world, vehicle, features, path, ss=False):
    # passing red traffic lights
    passing_trafficlight(vehicle=vehicle)

    # get at traffic light state
    at_traffic_light = vehicle.is_at_traffic_light()

    # ger lane information
    waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(
            carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
    lane_type = waypoint.lane_type
    left_lane = waypoint.left_lane_marking.type
    right_lane = waypoint.right_lane_marking.type

    # get control information
    control = vehicle.get_control()
    throttle, brake, steer = control.throttle, control.brake, control.steer

    # get location information
    location = vehicle.get_location()
    loc_x, loc_y = location.x, location.y

    # save images
    img_name = int(time.time() * 1000)

    if ss:
        cc = carla.ColorConverter.CityScapesPalette
        image.save_to_disk("%s/%d.png" % (path, img_name), cc)
    else:
        image.save_to_disk("%s/%d.png" % (path, img_name))

    features.append(["IMG/{}.png".format(str(img_name)), loc_x, loc_y, at_traffic_light, left_lane, right_lane, throttle, brake, steer])


def predict_steer2(image, model, control_unit, min_spd=2, max_spd=3, im_height=480, im_width=640):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (im_height, im_width, 4))
    array = array[:, :, :3]

    main_array = array[:, :, ::-1]
    img = tu.preprocess1(main_array)
    img = img.reshape((1, img.shape[0], img.shape[1], img.shape[2]))

    steer = model.predict(img)
    steer = float(steer[0][0])

    control_unit.auto_switch_02(steer, min_spd=min_spd, max_spd=max_spd)
    # control_unit.level_one(steer, min_spd=min_spd, max_spd=max_spd)


def predict_steer3(image, vehicle, model, csv_data, d=25, auto_switch=True, steer_limit=0.3, min_spd=2, max_spd=3, im_height=480, im_width=640):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (im_height, im_width, 4))
    array = array[:, :, :3]
    main_array = array[:, :, ::-1]
    img = tu.preprocess1(main_array)
    img = img.reshape((1, img.shape[0], img.shape[1], img.shape[2]))

    steer = model.predict(img)
    steer = float(steer[0][0])

    speed_limit = max_spd
    control = vehicle.get_control()
    v = vehicle.get_velocity()
    speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

    if speed > speed_limit:
        speed_limit = min_spd  # slow down
        throttle = 0.0
        brake = 1.0
    else:
        throttle = 1.0 - (steer ** 2) - (speed / speed_limit) ** 2
        speed_limit = max_spd
        brake = 0.0

    # location = vehicle.get_location()
    # loc_x = location.x
    # loc_y = location.y
    #
    # # Initialize variable that indicates whether the car is at the intersection or not
    # ent_int = False
    #
    # # check if vehicle is entering an intersection
    # for x, y in csv_data:
    #     if x - d <= loc_x <= x + d and y + d >= loc_y >= y - d:
    #         ent_int = True
    #         break

    passing_trafficlight(vehicle)

    if auto_switch:
        cond1 = abs(steer) > steer_limit
        # cond2 = ent_int

        if cond1:
            print("Level: 0 - SPDLIM: ", speed_limit)
            vehicle.set_autopilot(True)
        else:
            print("Level: 1 - SPDLIM: ", speed_limit)
            vehicle.set_autopilot(False)
            vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

    else:
        print("Level: 1 - SPDLIM: ", speed_limit)
        vehicle.set_autopilot(False)
        vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))


# -- Export waypoints --
def export_map_waypoints(world, csv_file, distance=2.0):
    print("Calling: exporting map waypoints (writing into map text file!)")
    # initialization
    map = world.get_map()
    waypoints_list = []
    map_waypoints = map.generate_waypoints(distance)

    # add waypoints to list
    for item in map_waypoints:
        loc = item.transform.location
        waypoints_list.append([loc.x, loc.y])

    if os.path.isfile(csv_file):
        print("- Removing existed file")
        os.remove(csv_file)

    # write coordinates to csv file
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        for loc_x, loc_y in waypoints_list:
            writer.writerow([loc_x, loc_y])


# -- Export spawn points --
def export_spawn_pts(world, csv_file):
    print("Calling: exporting spawn coordinates (writing into map text file!)")
    sp = world.get_map().get_spawn_points()
    sp_list = []

    for p in sp:
        x, y, z = p.location.x, p.location.y, p.location.z
        pitch, yaw, roll = p.rotation.pitch, p.rotation.yaw, p.rotation.roll
        sp_list.append([x, y, z, pitch, yaw, roll])

    if os.path.isfile(csv_file):
        print("- Removing existed file")
        os.remove(csv_file)

    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        for x, y, z, pitch, yaw, roll in sp_list:
            writer.writerow([x, y, z, pitch, yaw, roll])


# -- Sensor based: Position tracking -
def position_tracking(vehicle, pos_tracking, display=False):
    location = vehicle.get_location()
    pos_tracking.append([location.x, location.y])

    if display:
        print(location.x, location.y, location.z)


# -- Export lidar data ---
def export_lidar_csv(csv_file, features, loc_x, loc_y, theta, ms, com):
    # print("Calling: exporting vehicle data (writing into csv file!)")

    if os.path.isfile(csv_file):
        with open(csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            for item0, item1, item2, item3 in features:
                writer.writerow([item0, item1, item2, item3, loc_x, loc_y, theta, ms, com])
    else:
        with open(csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            for item0, item1, item2, item3 in features:
                writer.writerow([item0, item1, item2, item3, loc_x, loc_y, theta, ms, com])


def export_lidar_mpc_csv(csv_file, features, loc_x, loc_y, theta, ms, com, path, cy):
    # print("Calling: exporting vehicle data (writing into csv file!)")
    path = np.ndarray.flatten(path)

    if os.path.isfile(csv_file):
        with open(csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            for item0, item1, item2, item3 in features:
                content = [item0, item1, item2, item3, loc_x, loc_y, theta, ms, com]
                content.extend(path)
                content.extend(cy)
                writer.writerows([content])

    else:
        with open(csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            for item0, item1, item2, item3 in features:
                content = [item0, item1, item2, item3, loc_x, loc_y, theta, ms, com]
                content.extend(path)
                content.extend(cy)
                writer.writerows([content])


def random_waypoint(world, distance=2.0):
    map = world.get_map()
    waypoints_list = []
    map_waypoints = map.generate_waypoints(distance)


# == LiDAR functions =====
# -- LiDAR --
def display_lidar(point_cloud, vehicle, set_dir, lidar_range):
    passing_trafficlight(vehicle)
    try:
        # hud = np.array([1280, 720])
        hud = np.array([640, 480])

        # point cloud
        points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        # get location information
        location = vehicle.get_location()
        loc_x = location.x
        loc_y = location.y

        # vehicle speed
        ms, kmh = speed_estimation(vehicle)

        # get transform information
        degree = vehicle.get_transform().rotation.yaw
        rad = degree * np.pi / 180

        tag = int(time.time() * 1000)
        file = "{}/data_{}.csv".format(set_dir, tag)
        # export_lidar_csv(file, points, loc_x, loc_y, rad, ms, False)

        lidar_data = np.array(points[:, :2])
        lidar_data *= min(hud) / (2.0 * float(lidar_range))
        lidar_data += (0.5 * hud[0], 0.5 * hud[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))

        lidar_img_size = (hud[0], hud[1], 3)
        lidar_img = np.zeros(lidar_img_size, dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        cv2.imshow("lidar cam", lidar_img)
        cv2.waitKey(1)

    except IndexError as ie:
        print("Lidar: ", ie)

    except Exception as e:
        print("Lidar Exception: ", e)
        pass


def display_lidar_mpc(point_cloud, vehicle, set_dir, lidar_range, path, cy):
    try:
        # hud = np.array([1280, 720])
        hud = np.array([640, 480])

        # point cloud
        points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        # get location information
        location = vehicle.get_location()
        loc_x = location.x
        loc_y = location.y

        # vehicle speed
        ms, kmh = speed_estimation(vehicle)

        # get transform information
        degree = vehicle.get_transform().rotation.yaw
        rad = degree * np.pi / 180

        tag = int(time.time() * 1000)
        file = "{}/data_{}.csv".format(set_dir, tag)
        export_lidar_mpc_csv(file, points, loc_x, loc_y, rad, ms, False, path, cy)

        lidar_data = np.array(points[:, :2])
        lidar_data *= min(hud) / (2.0 * float(lidar_range))
        lidar_data += (0.5 * hud[0], 0.5 * hud[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))

        lidar_img_size = (hud[0], hud[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        cv2.imshow("lidar cam", lidar_img)
        cv2.waitKey(1)

    except IndexError as ie:
        print("Lidar: ", ie)

    except Exception as e:
        print("Lidar Exception: ", e)
        pass


def lidar_od(point_cloud, vehicle, stop_commands):
    # point cloud
    points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    data = strip_data(points, max_r=3, min_r=-1.5)

    # get location information
    location = vehicle.get_location()
    loc_x = location.x
    loc_y = location.y

    # get transform information
    degree = vehicle.get_transform().rotation.yaw
    rad = degree * np.pi / 180

    # vehicle speed
    ms, kmh = speed_estimation(vehicle)

    # vehicle bounding box
    cords = np.zeros((4, 2))
    extent = vehicle.bounding_box.extent
    cords[0, :] = np.array([extent.x, extent.y])
    cords[1, :] = np.array([-extent.x, extent.y])
    cords[2, :] = np.array([-extent.x, -extent.y])
    cords[3, :] = np.array([extent.x, -extent.y])

    tag = int(time.time() * 1000)
    # file = "lidar/lidar_data/{}/data_{}.csv".format(set_dir, tag)
    # export_lidar_csv(file, points, loc_x, loc_y, rad, cords)

    bdb = np.array([cords[2], cords[0]])

    factor = 0.8
    extent_d = 1
    region_length = 4.8 * factor * kmh
    region_width = 2.6 + extent_d * 2

    # Bottom left
    region_point1_x = bdb[0, 0] + 4.8
    region_point1_y = bdb[0, 1] - extent_d

    # Top right
    region_point2_x = bdb[1, 0] + region_length
    region_point2_y = bdb[1, 1] + extent_d

    region = np.array([region_point1_x, region_point1_y, region_point2_x, region_point2_y])

    pts = np.asarray([item[0:2] for item in data])
    on_region, off_region = [], []
    for pt in pts:
        obstacle = point_in_rectangle(pt, region)
        if obstacle:
            on_region.append([pt[0], pt[1]])
        else:
            off_region.append([pt[0], pt[1]])

    on_region, off_region = np.array(on_region), np.array(off_region)

    if len(on_region) > 0:
        stop_commands.append(True)
        # vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    else:
        stop_commands.append(False)
        # vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0, brake=0.0))

    print("- Lidar:", stop_commands[-1])


def lidar_od2(point_cloud, vehicle, stop_commands, stop_dist, set_dir):
    # point cloud
    points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    data = strip_data(points, max_r=1, min_r=-1.5)

    # get location information
    location = vehicle.get_location()
    loc_x = location.x
    loc_y = location.y

    # get transform information
    degree = vehicle.get_transform().rotation.yaw
    rad = degree * np.pi / 180

    # vehicle speed
    ms, kmh = speed_estimation(vehicle)

    # vehicle bounding box
    extent = vehicle.bounding_box.extent
    veh_tr_x = extent.x
    veh_tr_y = extent.y
    veh_bl_x = -veh_tr_x
    veh_bl_y = -veh_tr_y
    bb_length = abs(veh_tr_x - veh_bl_x)
    bb_width = abs(veh_tr_y - veh_bl_y)
    extent_d = 0.5
    min_distance = 6

    sd = stopping_dist(ms, factor=0.7)
    region_length = min_distance + sd
    region_width = bb_width + extent_d * 2

    # bottom left
    region_point1_x = veh_bl_x + bb_length
    region_point1_y = veh_bl_y - extent_d

    # top right
    region_point2_x = veh_tr_x + region_length
    region_point2_y = veh_tr_y + extent_d

    region = np.array([region_point1_x, region_point1_y, region_point2_x, region_point2_y])

    pts = np.asarray([item[0:2] for item in data])
    on_region, off_region = [], []
    for pt in pts:
        obstacle = point_in_rectangle(pt, region)
        if obstacle:
            on_region.append([pt[0], pt[1]])
        else:
            off_region.append([pt[0], pt[1]])

    on_region, off_region = np.array(on_region), np.array(off_region)

    if len(on_region) > 0:
        com = True
        stop_commands.append(com)
        # vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    else:
        com = False
        stop_commands.append(com)
        # vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0, brake=0.0))

    stop_dist.append(region_length)

    tag = int(time.time() * 1000)
    file = "{}/data_{}.csv".format(set_dir, tag)
    export_lidar_csv(file, points, loc_x, loc_y, rad, ms, com)


def save_lidar_data(data, path):
    data.save_to_disk('%s/%.6d.ply' % (path, data.frame))


def lidar_ss_function(point_cloud):
    points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 6), 6))
    print(points)


# == Collision sensor =====
def detect_collision(event, vehicle, actor_list):
    actor_we_collide_against = event.other_actor
    impulse = event.normal_impulse
    intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

    print("- Collision against: ", actor_we_collide_against)

    for actor in actor_list:
        actor.destroy()
    sys.exit()


def detect_invasion(event):
    crossed_lane_markings = event.crossed_lane_markings


# == GNSS functions =====
def gnss_function(event, gnss_data):
    lat, lon, alt = event.latitude, event.longitude, event.altitude
    tag = time.time()
    gnss_data.append([tag, lat, lon, alt])
    # if len(gnss_data) > 10:
    #     gnss_data.pop(0)
    # print("<LAT: {}> - <LON: {}> - <ALT: {}>".format(lat, lon, alt))


def check_enter_gps_int(event, gps_intersection):
    # Initialize variable that indicates whether the car is at the intersection or not
    current_latitude, current_longitude, alt = event.latitude, event.longitude, event.altitude
    ent_int = False

    # check if vehicle is entering an intersection
    for tag, lat, lon, dis in gps_intersection:
        lat_dis = km2latdeg(dis)
        lon_dis = km2londeg(dis, lat)
        if lat - lat_dis <= current_latitude <= lat + lat_dis and lon + lon_dis >= current_longitude >= lon - lon_dis:
            ent_int = True
            break

    print(ent_int)


def locate_intersection_coordinates(event, gnss_data, vehicle, intersection):
    # passing_trafficlight(vehicle)
    lat, lon, alt = event.latitude, event.longitude, event.altitude
    location = vehicle.get_location()
    loc_x, loc_y = location.x, location.y

    control = vehicle.get_control()
    throttle, brake, steer = control.throttle, control.brake, control.steer

    enter_intersection = check_enter_intersection_data(intersection, loc_x, loc_y)
    ms, kmh = speed_estimation(vehicle)

    cond1 = brake == 1
    cond2 = round(kmh, 1) == 0.0
    cond3 = enter_intersection

    if cond1 or cond2:
        tag = int(time.time() * 1000)
        gnss_data.append([tag, lat, lon, alt])


def imu_function(sensor_data, gnss_data, imu_data):
    limits = (-99.9, 99.9)
    accelerometer = (
        max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
        max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
        max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
    gyroscope = (
        max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
        max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
        max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
    compass = math.degrees(sensor_data.compass)

    # print("<ACCE: {}> - <GYRO: {}> - <COMP: {}>".format(accelerometer, gyroscope, compass))

    if len(gnss_data) > 0:
        tag = gnss_data[-1][0]
        a0, a1, a2 = accelerometer[0], accelerometer[1], accelerometer[2]
        g0, g1, g2 = gyroscope[0], gyroscope[1], gyroscope[2]
        imu_data.append([tag, a0, a1, a2, g0, g1, g2, compass])


# == HELP FUNCTIONS
# -- Passing red lights --------
def passing_trafficlight(vehicle):
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            # world.hud.notification("Traffic light changed! Good to go!")
            traffic_light.set_state(carla.TrafficLightState.Green)


def latdeg2km(lat):
    km = lat * 110.574
    return km


def londeg2km(lat_coordinate, lon):
    factor = 111.320 * math.cos(lat_coordinate)
    km = lon * factor
    return km


def km2latdeg(km):
    lat = km / 110.574
    return lat


def km2londeg(km, lat_coordinate):
    factor = 111.320 * math.cos(lat_coordinate)
    lon = km / factor
    return lon


def gps_distance(lat1, lon1, lat2, lon2):
    # radius of the Earth
    R = 6373.0

    # coordinates
    lat1 = np.deg2rad(lat1)
    lon1 = np.deg2rad(lon1)
    lat2 = np.deg2rad(lat2)
    lon2 = np.deg2rad(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.atan2(np.sqrt(a), np.sqrt(1 - a))

    distance = R * c

    print("Result:", distance)


def point_in_rectangle(point, rectangle):
    x, y = point
    x1, y1, x2, y2 = rectangle

    if x1 <= x <= x2 and y1 <= y <= y2:
        return True
    else:
        return False


def strip_data(data, max_r=1, min_r=-0.5):
    return [item for item in data if max_r > item[2] > min_r]


def check_enter_intersection_data(data, loc_x, loc_y):
    # Initialize variable that indicates whether the car is at the intersection or not
    ent_int = False

    # check if vehicle is entering an intersection
    for num, x, y, d in data:
        if x - d <= loc_x <= x + d and y + d >= loc_y >= y - d:
            ent_int = True
            break

    return ent_int


# -- Speed estimation
def speed_estimation(vehicle):
    v = vehicle.get_velocity()
    ms = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    kmh = int(3.6 * ms)
    print('Speed: %.4f (m/s) - %.4f (km/h)' % (ms, kmh))

    return ms, kmh


def stopping_dist(v, factor=1.0, vf=2.0):
    ro = 1.225
    m = 1845
    Cd = 0.3
    friction = 0.7

    Ka = (ro / (2 * m)) * Cd
    Kt = (0.01 + friction) * 9.80665
    d = (1 / (2 * Ka)) * np.log(1 + (Ka / Kt) * (v ** vf))
    d = d * factor

    return d


def lane_data(data, vehicle, world):
    # pass traffic light
    passing_trafficlight(vehicle)

    # lane information
    waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(
                carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))

    print("==" * 50)
    print("Current lane type: " + str(waypoint.lane_type))
    # Check current lane change allowed
    print("Current Lane change:  " + str(waypoint.lane_change))
    # Left and Right lane markings
    print("L lane marking type: " + str(waypoint.left_lane_marking.type))
    print("R lane marking type: " + str(waypoint.right_lane_marking.type))
    print(waypoint.lane_type, waypoint.left_lane_marking.type, waypoint.right_lane_marking.type)

    # spectator
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                            carla.Rotation(pitch=-90)))

