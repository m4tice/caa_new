import numpy as np
import caa.c_utils.data_util as du
import caa.c_utils.training_util as tu

import carla
import math
import cv2


class E2EController:
    def __init__(self, vehicle, model, min_spd, max_spd, carla_int_csv, gps_int_csv, cam_set):
        print("<!> A Control Unit is created for:", vehicle.type_id)
        self.vehicle = vehicle
        self.model = model
        self.carla_intersections = du.load_intersection_data(carla_int_csv)
        self.gps_intersections = du.load_intersection_data(gps_int_csv)
        self.im_height = int(cam_set['image_size_y'])
        self.im_width = int(cam_set['image_size_x'])
        self.min_spd = min_spd
        self.max_spd = max_spd
        self.speed_limit = self.max_spd
        self.image = None
        self.steer = 0
        self.throttle = None
        self.brake = None
        self.level = None
        self.ent_int = None
        self.lidar_detection = None
        self.nearest_dist = None
        self.npc_autopilot = False
        self.v = self.vehicle.get_velocity()
        self.speed = math.sqrt(self.v.x ** 2 + self.v.y ** 2 + self.v.z ** 2)

    # Build image from raw data
    def build_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (self.im_height, self.im_width, 4))
        array = array[:, :, :3]

        return array

    def predict_steering(self, image):
        self.image = self.build_image(image)

        main_array = self.image[:, :, ::-1]
        img = tu.preprocess1(main_array)
        img = img.reshape((1, img.shape[0], img.shape[1], img.shape[2]))

        steer = self.model.predict(img)
        steer = float(steer[0][0])

        return steer

    def check_enter_carla_int(self, carla_intersection):
        # Get the current location of the vehicle
        location = self.vehicle.get_location()
        loc_x, loc_y = location.x, location.y

        # Initialize variable that indicates whether the car is at the intersection or not
        ent_int = False

        # check if vehicle is entering an intersection
        for num, x, y, d in carla_intersection:
            if x - d <= loc_x <= x + d and y + d >= loc_y >= y - d:
                ent_int = True
                break

        return ent_int

    def check_enter_gps_int(self, gps_intersection, gnss_data):
        # Initialize variable that indicates whether the car is at the intersection or not
        current_data = gnss_data[-1]
        current_latitude = current_data[1]
        current_longitude = current_data[2]
        ent_int = False

        # check if vehicle is entering an intersection
        for tag, lat, lon, dis in gps_intersection:
            lat_dis = self.km2latdeg(dis)
            lon_dis = self.km2londeg(dis, current_latitude)
            if lat - lat_dis <= current_latitude <= lat + lat_dis and lon + lon_dis >= current_longitude >= lon - lon_dis:
                ent_int = True
                break

        return ent_int

    def steering_to_throttle(self, steering):
        self.v = self.vehicle.get_velocity()
        self.speed = math.sqrt(self.v.x ** 2 + self.v.y ** 2 + self.v.z ** 2)

        if self.speed > self.max_spd:
            self.speed_limit = self.min_spd  # slow down
            throttle = 0.0
            brake = 1.0
        else:
            throttle = 1.0 - (steering ** 2) - (self.speed / self.speed_limit) ** 2
            self.speed_limit = self.max_spd
            brake = 0.0

        return throttle, brake

    def passing_trafficlight(self):
        if self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                traffic_light.set_state(carla.TrafficLightState.Green)

    def show_opencv_window(self, pp1=False):
        main_array = self.image
        if pp1:
            main_array = self.image[:, :, ::-1]
            main_array = tu.preprocess1(main_array)

        cv2.imshow("front_cam", main_array)
        cv2.waitKey(1)

    def level_zero(self):
        self.passing_trafficlight()
        self.v = self.vehicle.get_velocity()
        self.speed = math.sqrt(self.v.x ** 2 + self.v.y ** 2 + self.v.z ** 2)

    def level_one(self, image, camera=False, pp1=False):
        self.steer = self.predict_steering(image)
        self.throttle, self.brake = self.steering_to_throttle(self.steer)
        # self.vehicle.apply_control(carla.VehicleControl(throttle=self.throttle, steer=self.steer, brake=self.brake))

        # Display camera view
        if camera:
            self.show_opencv_window(pp1=pp1)

        return self.steer, self.throttle, self.brake