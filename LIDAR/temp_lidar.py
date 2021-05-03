import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import math

from datetime import datetime
from caa_new.LIDAR.temp_lidar_2 import is_inside_polygon


data_dic = {
    'x'        : 0,
    'y'        : 1,
    'z'        : 2,
    'intensity': 3,
    'loc_x'    : 4,
    'loc_y'    : 5,
    'theta'    : 6,
    'ms'       : 7,
    'com'      : 8,
    'p0'       : 9,
    'p1'       :10,
    'p2'       :11,
    'p3'       :12,
    'p4'       :13,
    'p5'       :14,
    'p6'       :15,
    'p7'       :16,
    'p8'       :17,
    'p9'       :18,
    'p10'      :19,
    'p11'      :20,
    'p12'      :21,
    'p13'      :22,
    'p14'      :23,
    'p15'      :24,
    'p16'      :25,
    'p17'      :26,
    'cy0'      :27,
    'cy1'      :28,
    'cy2'      :29,
    'cy3'      :30,
    'cy4'      :31,
    'cy5'      :32,
    'cy6'      :33,
    'cy7'      :34,
    'cy8'      :35,
}


def load_map(csv_map):
    print("Calling: loading map")
    map_data_df = pd.read_csv(csv_map, names=['x', 'y'])
    waypoints = map_data_df[['x', 'y']].values

    return waypoints


def load_lidar_data(csv_file):
    data_df = pd.read_csv(csv_file, names=['x', 'y', 'z', 'intensity', 'loc_x', 'loc_y', 'theta', 'ms', 'com',
                                           'p0', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7',
                                           'p8', 'p9', 'p10', 'p11', 'p12', 'p13', 'p14', 'p15',
                                           'p16', 'p17', 'cy0', 'cy1', 'cy2', 'cy3', 'cy4', 'cy5', 'cy6', 'cy7', 'cy8'])
    data = data_df[['x', 'y', 'z', 'intensity', 'loc_x', 'loc_y', 'theta', 'ms', 'com',
                    'p0', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7',
                    'p8', 'p9', 'p10', 'p11', 'p12', 'p13', 'p14', 'p15',
                    'p16', 'p17', 'cy0', 'cy1', 'cy2', 'cy3', 'cy4', 'cy5', 'cy6', 'cy7', 'cy8']].values

    return data


waypoints_csv = "../database/town02/town02-waypoints.csv"
wp = load_map(waypoints_csv)
fig, ax = plt.subplots()
# ax.set_facecolor('gray')
ax.axis('equal')

csv_path = "../collected_data/lidar/town02/set_02"
img_path = "../collected_data/lidar/town02/set_02_img"
files = os.listdir(csv_path)


def strip_data(data, max_r=1, min_r=-1.5):
    return [item for item in data if max_r > item[2] > min_r]


def transform_distance(p1, p2):
    return p1[0]-p2[0], p1[1]-p2[1]


def translate(p, Tx, Ty):
    px = p[0] + Tx
    py = p[1] + Ty

    return [px, py]


def rotate(x, y, xo, yo, theta):  # rotate x,y around xo,yo by theta (rad)
    xr = math.cos(theta) * (x - xo) - math.sin(theta) * (y - yo) + xo
    yr = math.sin(theta) * (x - xo) + math.cos(theta) * (y - yo) + yo
    return [xr, yr]


bx = 2.396
by = 1.082

tr = [bx, by]
tl = [-bx, by]
br = [-bx, -by]
bl = [bx, -by]
bb = np.array([tr, tl, br, bl])


for file_name in files[100:101]:
    start_time = datetime.now()
    ax.cla()
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])

    file = os.path.join(csv_path, file_name)
    # data = load_lidar_data(file)
    df = pd.read_csv(file, header=None)
    data = df.values
    data = strip_data(data, 1, -1.5)

    # Add ~0.5 seconds
    data = [item for item in data if not is_inside_polygon(bb, (item[0], item[1]))]

    data = np.array(data)

    points = data[:, :3]
    ex, ey = data[0][4], data[0][5]
    theta = data[0][6]
    path = data[0][9:43]
    path = np.reshape(path, (17, 2))
    cys = data[0][43:]

    Tx, Ty = transform_distance([0, 0], [ex, ey])
    org = translate([ex, ey], Tx, Ty)

    path = np.array([translate(item, Tx, Ty) for item in path])
    path = np.array([rotate(item[0], item[1], org[0], org[1], -theta) for item in path])

    ax.scatter(points[:, 0], points[:, 1], marker='.')

    p1, p2 = [], []
    for p, cy in zip(path, cys):
        temp_line = np.array([[p[0], p[1] + by + 0.5], [p[0], p[1] - by - 0.5]])
        temp_line = np.array([rotate(item[0], item[1], p[0], p[1], cy) for item in temp_line])
        temp_line = np.array([rotate(item[0], item[1], p[0], p[1], -theta) for item in temp_line])
        p1.append(temp_line[0])
        p2.append(temp_line[1])

    p2.reverse()
    p1.extend(p2)
    p1arr = np.array(p1)
    ax.plot(p1arr[:, 0], p1arr[:, 1], color='green')

    p3 = np.array([[0, 0 + by + 0.5], [bx * 3, 0 + by + 0.5], [bx * 3, 0 - by - 0.5], [0, 0 - by - 0.5]])
    ax.plot(p3[:, 0], p3[:, 1], color='orange')

    ax.plot(bb[:, 0], bb[:, 1], color='blue')

    shorten_points = np.array([point for point in points if point[0] > 0])

    in_region1 = [[point[0], point[1]] for point in shorten_points if is_inside_polygon(p1, (point[0], point[1]))]
    in_region2 = [[point[0], point[1]] for point in shorten_points if is_inside_polygon(p3, (point[0], point[1]))]

    in_region1 = np.array(in_region1)
    in_region2 = np.array(in_region2)

    # if len(in_region1) > 0:
    #     ax.scatter(in_region1[:, 0], in_region1[:, 1], color='purple')

    if len(in_region2) > 0:
        ax.scatter(in_region2[:, 0], in_region2[:, 1], color='pink')

    plt.gca().invert_yaxis()
    print("Processing time: {}".format(datetime.now() - start_time))

    file_name = file_name.split('.')
    file_name = file_name[0]
    # plt.savefig(os.path.join(img_path, "lidar_{}.png".format(file_name)))
    plt.pause(0.000001)

plt.show()
