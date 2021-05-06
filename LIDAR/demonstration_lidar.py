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
import math
import keyboard
import warnings
warnings.filterwarnings("ignore")
import cvxpy
import cv2

import numpy as np

sys.path.append(r"../../")
from caa_new.my_utils import dictionaries as dic
from caa_new.my_utils import sensor_util as su
from caa_new.my_utils import tools as to
from caa_new.GlobalPathPlanning import module_p2

from caa_new.E2E.module_e2e import E2EController
from caa_new.MPC import cubic_spline_planner as cubic_spline_planner
from caa_new.GNSS import module_gnss
from caa_new.LIDAR import module_lidar
from tensorflow.keras.models import load_model

from queue import Queue
from queue import Empty
from datetime import datetime

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

# -- MPC INITIALIZATION -------
# == Model Predictive Controller =======================================================================================
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 16  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 36 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.05  # [s] time tick

# Vehicle parameters
# LENGTH = 4.5  # [m]
# WIDTH = 2.0  # [m]
# BACKTOWHEEL = 1.0  # [m]
# WHEEL_LEN = 0.3  # [m]
# WHEEL_WIDTH = 0.2  # [m]
# TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(69.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(69.0)  # maximum steering speed [rad/s]
MAX_SPEED = 36 / 3.6  # maximum speed [m/s]
MIN_SPEED = 0.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    while angle > math.pi:
        angle = angle - 2.0 * math.pi

    while angle < -math.pi:
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, a, delta):
    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])

    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        # state = update_carla_state(vehicle, state)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)

        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)

        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        pass
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


# Broken
def check_goal(state, goal, tind, nind):
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isstop:
        return True

    return False


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def carla_world_plot(world, path, z=1.5, lt=0.0):
    debug = world.debug
    for x, y in path:
        location = carla.Location(x=x, y=y, z=z)
        debug.draw_point(
            location,
            size=0.1,
            life_time=lt,
            persistent_lines=False,
            color=carla.Color(0, 255, 0))


def get_course(path, dl=1.0):
    ax, ay = path[:, 0], path[:, 1]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def update_carla_state(vehicle, state, delta):
    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    # get location
    location = vehicle.get_location()
    loc_x, loc_y = location.x, location.y

    # get rotation
    yaw = vehicle.get_transform().rotation.yaw
    yaw = np.deg2rad(yaw)

    # get velocity
    v = vehicle.get_velocity()
    ms = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    kmh = int(3.6 * ms)

    state.x = loc_x
    state.y = loc_y
    state.yaw = yaw
    state.v = kmh

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state, delta


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data, sensor_name))


def mpc_drive(world, vehicle, state, cx, cy, cyaw, ck, sp, dl, target_ind):
    xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)
    xr = xref[0]
    yr = xref[1]
    path = [[x, y] for x, y in zip(xr, yr)]
    path = np.asarray(path)

    carla_world_plot(world, path)

    x0 = [state.x, state.y, state.v, state.yaw]  # current state
    oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

    if odelta is not None:
        di, ai = odelta[0], oa[0]

    vehicle.apply_control(carla.VehicleControl(throttle=ai, steer=di, brake=0))


def game_loop(reload=True, hp=False, cp=False):  # hp: Horizon plot - cp: Course plot
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Using module Global Path Planning to get a predefined course
    course = module_p2.get_course(road_segments_file, dic.course_01)
    # course = course[85:]

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_course(course, dl)
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    initial_state = State(x=cx[0], y=cy[0], yaw=np.rad2deg(cyaw[0]), v=0.0)

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
        world.unload_map_layer(carla.MapLayer.Buildings)
        world.unload_map_layer(carla.MapLayer.Props)
        world.unload_map_layer(carla.MapLayer.StreetLights)
        blueprint_library = world.get_blueprint_library()  # get blueprint library

        if cp:
            carla_world_plot(world, course)  # Toggle to plot the

        # Vehicle
        vehicle_model = "mercedesccc"  # "model3"/"lincoln"/"mercedesccc"
        vehicle_color = dic.petronas_color
        vehicle = su.spawn_mpc_vehicle(world, blueprint_library, initial_state, vehicle_model, vehicle_color)
        actor_list.append(vehicle)

        # Vehicle
        vehicle2_model = "lincoln"  # "model3"/"lincoln"/"mercedesccc"
        vehicle2_color = dic.orange
        vehicle2 = su.spawn_vehicle(world, blueprint_library, vehicle2_model, vehicle2_color, spawn_number=10)
        actor_list.append(vehicle2)

        # Get Vehicle Physics Control
        physics_control = vehicle.get_physics_control()

        # For each Wheel Physics Control, print maximum steer angle
        for wheel in physics_control.wheels:
            print(">>>", wheel.max_steer_angle)

        # Controller E2E
        print('- Loading model...')
        model_name = '../models/town02/model-t2s4-050-0.000090.h5'
        model = load_model(model_name)
        print("- Using model: {}".format(model_name))

        controller = E2EController(vehicle, model, min_spd=2, max_spd=6,
                                   carla_int_csv=carla_intersection_csv,
                                   gps_int_csv=gps_intersection_csv,
                                   cam_set=dic.cam_rgb_set_2)

        # Controller MPC
        goal = [cx[-1], cy[-1]]
        state = initial_state

        # initial yaw compensation
        if state.yaw - cyaw[0] >= math.pi:
            state.yaw -= math.pi * 2.0
        elif state.yaw - cyaw[0] <= -math.pi:
            state.yaw += math.pi * 2.0

        time_step = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

        odelta, oa = None, None
        cyaw = smooth_yaw(cyaw)

        # == Camera: RGB
        cam_rgb = su.spawn_camera_rgb(world, blueprint_library, vehicle, dic.cam_rgb_set_2)
        cam_rgb.listen(lambda data: sensor_callback(data, sensor_queue, 1))
        sensor_list.append(cam_rgb)

        # == Camera: spectator
        cam_spectate = su.spawn_camera_rgb(world, blueprint_library, vehicle, dic.cam_spectate_1)
        cam_spectate.listen(lambda data: sensor_callback(data, sensor_queue, 2))
        sensor_list.append(cam_spectate)

        # == Lidar:
        lidar = su.spawn_lidar(world, blueprint_library, vehicle, dic.lidar)
        lidar.listen(lambda data: sensor_callback(data, sensor_queue, 3))
        sensor_list.append(lidar)

        # Sensor listening
        print("Stage: listening to sensor")
        while True:
            world.tick()
            print("\n== DRIVING LOG =====================================")
            start_time = datetime.now()
            try:
                # Display speed information
                ms, kmh = su.speed_estimation(vehicle)

                # mpc
                xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)
                xr = xref[0]
                yr = xref[1]
                cyr = xref[3]
                path = [[x, y] for x, y in zip(xr, yr)]
                path = np.asarray(path)

                if hp:
                    carla_world_plot(world, path, lt=1.5)

                x0 = [state.x, state.y, state.v, state.yaw]  # current state
                oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

                if odelta is not None:
                    di, ai = odelta[0], oa[0]

                state, di = update_carla_state(vehicle, state, di)

                for _ in range(len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    if s_frame[1] == 1:
                        # Using E2E controller to get the steer and throttle information
                        v_steer, v_throttle, _ = controller.level_one(s_frame[0], camera=False, pp1=True)

                    elif s_frame[1] == 2:
                        # Streaming from spectator camera
                        su.live_cam(s_frame[0], dic.cam_spectate_1)

                    elif s_frame[1] == 3:
                        # Save lidar data
                        # lidar_set_name = "../collected_data/lidar/town02/set_02"
                        # display_lidar(s_frame[0], vehicle, lidar_set_name, dic.lidar['range'], path, cyr)

                        # Lidar detection stopping module
                        v_brake = module_lidar.lidar_detection(s_frame[0], vehicle, path, cyr)

                print("THROTTLE: {}  - STEER: {} - BRAKE: {}".format(v_throttle, v_steer, v_brake))
                vehicle.apply_control(carla.VehicleControl(throttle=v_throttle, steer=v_steer, brake=v_brake))

                # Stopping key
                if keyboard.is_pressed("q"):
                    print("Simulation stopped")
                    break

            except Empty:
                print("- Some of the sensor information is missed")

            print("Total processing time:", datetime.now()-start_time)

    finally:
        print("Finally...")

        # Switch back to synchronous mode
        settings.synchronous_mode = False
        world.apply_settings(settings)
        try:
            # Destroying actors
            print("Destroying {} actor(s)".format(len(actor_list)))
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

        except Exception as e:
            print("Final Exception: ", e)


def main():
    game_loop(reload=True, hp=True, cp=False)
    time.sleep(0.5)


if __name__ == '__main__':
    main()
