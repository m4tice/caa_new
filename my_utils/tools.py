import os
import pandas as pd
import matplotlib.pyplot as plt


# == INITIALIZATION =========
def paths_initialization(town_dic):
    main_path = '../'
    town_dir = town_dic['dir_name']
    map_name = town_dic['name']
    weather_type = town_dic['weather']

    intersection_csv = os.path.join(main_path, 'database', town_dir, '{}-intersections.csv'.format(town_dir))
    waypoints_csv = os.path.join(main_path, 'database', town_dir, '{}-waypoints.csv'.format(town_dir))
    spawn_csv = os.path.join(main_path, 'database', town_dir, '{}-spawn-locations.csv'.format(town_dir))
    gps_intersection_csv = os.path.join(main_path, 'database', town_dir, '{}-gps-intersections.csv'.format(town_dir))
    road_segments = os.path.join(main_path, 'database', town_dir, '{}_road_segments.txt'.format(town_dir))

    collected_data_dir = os.path.join(main_path, "collected_data")

    img_dir = os.path.join(collected_data_dir, "camera_rgb/IMG")
    csv_file = os.path.join(collected_data_dir, "camera_rgb/driving_log.csv")
    lidar_dir = os.path.join(collected_data_dir, "lidar")
    gnss_csv = os.path.join(collected_data_dir, "gnss_data.csv")

    print("== PATHS INITIALIZATION RESULTS =========================")
    if not os.path.isfile(intersection_csv):
        print("Missing              :", '{}-intersections.csv'.format(town_dir))
        intersection_csv = None

    if not os.path.isfile(waypoints_csv):
        print("Missing              :", '{}-waypoints.csv'.format(town_dir))
        waypoints_csv = None

    if not os.path.isfile(spawn_csv):
        print("Missing              :", '{}-spawn-locations.csv'.format(town_dir))
        spawn_csv = None

    if not os.path.isfile(gps_intersection_csv):
        print("Missing              :", '{}-gps-intersections.csv'.format(town_dir))
        gps_intersection_csv = None

    if not os.path.isfile(road_segments):
        print("Missing              :", '{}_road_segments.txt'.format(town_dir))
        road_segments = None

    if not os.path.isdir(img_dir):
        print("'IMG' directory      : newly created")
        os.mkdir(img_dir)
    else:
        print("'IMG' directory      : using existed")

    if not os.path.isdir(lidar_dir):
        print("'lidar' directory    : newly created")
        os.mkdir(lidar_dir)
    else:
        print("'lidar' directory    : using existed")

    if not os.path.isfile(csv_file):
        print("'driving_log.csv'    : not existing")
    else:
        print("'driving_log.csv'    : using existed")

    if not os.path.isfile(gnss_csv):
        print("'gnss_data.csv'      : not existing")
    else:
        print("'gnss_data.csv'      : using existed")

    results = [map_name, weather_type, town_dir, img_dir, csv_file, lidar_dir, gnss_csv,
               gps_intersection_csv, waypoints_csv, spawn_csv, intersection_csv, road_segments]

    status = "-- STATUS            : Passed"
    for res in results:
        if res is None:
            status = "-- STATUS            : Partly completed"
            break

    print(status)

    return results


# -- Load town map -----
def load_map(csv_map):
    print("Calling: loading map")
    map_data_df = pd.read_csv(csv_map, names=['x', 'y'])
    waypoints = map_data_df[['x', 'y']].values

    return waypoints


# -- Load intersections coordinates -----
def load_intersection_data(csv_coors):
    data_df = pd.read_csv(csv_coors, names=['number', 'x', 'y', 'r'])
    coors = data_df[['number', 'x', 'y', 'r']].values

    return coors


# -- Load spawn points -----
def load_spawn_points_data(csv_pts):
    data_df = pd.read_csv(csv_pts, names=['x', 'y', 'z', 'pitch', 'yaw', 'roll'])
    pts = data_df[['x', 'y', 'z', 'pitch', 'yaw', 'roll']].values

    return pts


def plot_spawn_points(fig, ax, pts):
    spx = pts[:, 0]
    spy = pts[:, 1]

    ax.scatter(spx, spy, color='black', marker='+')

    for i, p in enumerate(pts):
        ax.text(p[0], p[1]+1, i, color='black')

        
# -- Plot map ----------
def map_plot(csv_map, csv_pts):
    print("Calling: showing map")
    offset = 20

    waypoints = load_map(csv_map)
    mxs = waypoints[:, 0]
    mys = waypoints[:, 1]

    pts = load_spawn_points_data(csv_pts)

    fig1, ax1 = plt.subplots()
    ax1.axis('equal')
    ax1.grid(True)
    ax1.axis([min(mxs) - offset, max(mxs) + offset, min(mys) - offset, max(mys) + offset])
    ax1.set_facecolor('gray')
    ax1.scatter(mxs, mys, color='white', marker='.')

    plot_spawn_points(fig1, ax1, pts)
    plt.gca().invert_yaxis()
