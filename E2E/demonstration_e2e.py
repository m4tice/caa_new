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

import time
import keyboard
import numpy as np

from tensorflow.keras.models import load_model
from queue import Queue
from queue import Empty

sys.path.append(r"../../")
from caa_new.my_utils import dictionaries as dic
from caa_new.my_utils import sensor_util as su
from caa_new.my_utils import tools as to
from caa_new.E2E.module_e2e import E2EController


np.random.seed(2)

# -- VARIABLE INITIALIZATION --
# Specify the wanted map
town_dic = dic.town02

map_name, \
    weather_type, \
    current_town, \
    img_dir, \
    csv_file, \
    lidar_dir, \
    gnss_csv, \
    gps_intersection_csv, \
    waypoints_csv, \
    spawn_csv, \
    carla_intersection_csv, \
    road_segments_file = to.paths_initialization(town_dic)


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data, sensor_name))


def game_loop(reload=True):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # List initialization
    actor_list, sensor_list, features = [], [], []
    sensor_queue = Queue()

    try:
        if reload:
            print("Loading world...")
            world = client.load_world(town_dic['name'])
        else:
            world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        world.apply_settings(settings)

        # Spectator
        spectator = world.get_spectator()
        transform = carla.Transform(carla.Location(x=town_dic['x'], y=town_dic['y'], z=town_dic['z']),
                                    carla.Rotation(pitch=town_dic['pitch'], yaw=town_dic['yaw']))
        spectator.set_transform(transform)

        # World setting
        world.set_weather(getattr(carla.WeatherParameters, town_dic['weather']))  # set weather
        blueprint_library = world.get_blueprint_library()  # get blueprint library

        # Vehicle
        vehicle_model = "mercedesccc"  # "model3"/"lincoln"/"mercedesccc"
        vehicle_color = dic.petronas_color
        vehicle = su.spawn_vehicle(world, blueprint_library, vehicle_model, color=vehicle_color, spawn_number=None)
        actor_list.append(vehicle)

        # Prediction Model
        print('- Loading model...')
        model_name = '../models/town02/model-t2s4-050-0.000090.h5'
        model = load_model(model_name)
        print("- Using model: {}".format(model_name))

        # Controller
        controller = E2EController(vehicle, model, min_spd=3, max_spd=8,
                                   carla_int_csv=carla_intersection_csv,
                                   gps_int_csv=gps_intersection_csv,
                                   cam_set=dic.cam_rgb_set_2)

        # == Camera: RGB
        cam_rgb = su.spawn_camera_rgb(world, blueprint_library, vehicle, dic.cam_rgb_set_2)
        cam_rgb.listen(lambda data: sensor_callback(data, sensor_queue, 1))
        sensor_list.append(cam_rgb)

        # == Camera: spectator
        cam_spectate = su.spawn_camera_rgb(world, blueprint_library, vehicle, dic.cam_spectate_1)
        cam_spectate.listen(lambda data: sensor_callback(data, sensor_queue, 2))
        sensor_list.append(cam_spectate)

        # Sensor listening
        print("Stage: listening to sensor")
        while True:
            world.tick()
            try:
                for _ in range(len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    if s_frame[1] == 1:
                        controller.level_one(s_frame[0], camera=True, pp1=True)
                    else:
                        su.live_cam(s_frame[0], dic.cam_spectate_1)

                # Stopping key
                if keyboard.is_pressed("q"):
                    print("Simulation stopped")
                    break

            except Empty:
                print("- Some of the sensor information is missed")

    finally:
        print("Finally...")

        # Switch back to synchronous mode
        settings.synchronous_mode = False
        world.apply_settings(settings)
        try:
            # Destroying actors
            print("Destroying {} actor(s)".format(len(actor_list)))
            client.apply_batch([carla.command.DestroyActor(x) for x in sensor_list])
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

        except Exception as e:
            print("Final Exception: ", e)


def main():
    game_loop(reload=True)
    time.sleep(0.5)


if __name__ == '__main__':
    main()
