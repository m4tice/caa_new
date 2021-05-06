import math
import random

import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime
from caa_new.GlobalPathPlanning.dijkstra import shortest_path, Graph
from caa_new.my_utils import dictionaries as dic


def compare_segments(road1, road2):
    road1_xs, road1_ys = road1
    road2_xs, road2_ys = road2

    xs_compare = all(map(lambda x, y: x == y, road1_xs, road2_xs))
    ys_compare = all(map(lambda x, y: x == y, road1_ys, road2_ys))

    if xs_compare and ys_compare:
        return True
    else:
        return False


def compute_distance(p1, p2):
    distance = math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))
    return distance


def clean_road_segments(road_segments_file):
    f = open(road_segments_file, "r")
    lines = f.readlines()
    data = []

    for line in lines:
        content = line.split(",")
        content = [float(item) for item in content]
        data.append(content)

    segments = []

    for i in range(0, len(data), 2):
        segments.append([data[i], data[i + 1]])

    reduced_segments = [segments[0]]

    for road1 in segments:
        comparison_list = [compare_segments(road1, road2) for road2 in reduced_segments]
        found = any(comparison_list)

        if not found:
            reduced_segments.append(road1)

    for road in reduced_segments:
        road[0].pop(0)
        road[0].pop(-1)
        road[1].pop(0)
        road[1].pop(-1)

    return reduced_segments


def nodes_initialization(node_points, name='t02-n'):
    node_dict, node_names = [], []

    for i in range(len(node_points)):
        # idx = "{:05d}".format(i)
        node_name = '{}{:05d}'.format(name, i)
        node_names.append(node_name)

    for point, name in zip(node_points, node_names):
        node_dict.append([point, name])

    node_dict = np.asarray(node_dict)

    return node_dict, node_names


def get_node_name(node_dict, point):
    x, y = point
    index = None

    for idx, node in enumerate(node_dict):
        nx, ny = node[0]
        if nx == x and ny == y:
            return node_dict[idx][1]


def get_node_point(node_dict, name):
    for idx, node in enumerate(node_dict):
        node_name = node[1]
        if node_name == name:
            return node_dict[idx][0]


def convert_points_to_name(node_dict, points_con):
    new_point_con = []

    for idx, (point_1, point_2, distance) in enumerate(points_con):
        name_1, name_2 = get_node_name(node_dict, point_1), get_node_name(node_dict, point_2)
        new_point_con.append([name_1, name_2, distance])

    return new_point_con


def create_nodes(segments):
    nodes = []
    for road in segments:
        for i in range(len(road[0])):
            nodes.append([road[0][i], road[1][i]])

    return nodes


def nodes_to_path(node_dict, nodes):
    path = []
    for node in nodes:
        point = get_node_point(node_dict, node)
        path.append(point)

    return path


def create_connection_end_nodes(segments, lower_lim=0, upper_lim=2):
    start_nodes, end_nodes = [], []

    for seg in segments:
        if len(seg[0]) == 0 or len(seg[1]) == 0:
            pass
        else:
            start_node = seg[0][0], seg[1][0]
            end_node = seg[0][-1], seg[1][-1]

            start_nodes.append(start_node)
            end_nodes.append(end_node)

        connection_list = []

        for p1 in end_nodes:
            for p2 in start_nodes:
                distance = math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))
                if lower_lim < distance <= upper_lim:
                    connection_list.append([p1, p2, distance])

    return connection_list


def create_connection_segment_nodes(segments, lower_lim=0, upper_lim=1.9):
    connection_list = []
    for road in segments:
        amount = len(road[0])

        for i in range(amount):
            for j in range(i, amount):
                point1 = [road[0][i], road[1][i]]
                point2 = [road[0][j], road[1][j]]
                distance = compute_distance(point1, point2)
                if lower_lim < distance < upper_lim:
                    connection_list.append([point1, point2, distance])

    return connection_list


# return a random path: random start and random destination
def get_random_course(road_segments_file, noc=4, demo=False, time_analysis=False):
    if time_analysis:
        print("\n== DIJKSTRA TIME ANALYSIS ===============================")
        start_time = datetime.now()

    segments = clean_road_segments(road_segments_file)
    node_points = create_nodes(segments)
    node_points = np.asarray(node_points)

    node_dict, node_names = nodes_initialization(node_points, name='t02-w')

    connections_1 = create_connection_end_nodes(segments, lower_lim=0, upper_lim=4)

    connections_2 = create_connection_segment_nodes(segments, lower_lim=0, upper_lim=2.1)
    connections = connections_1 + connections_2

    graph = Graph()

    for node in node_names:
        graph.add_node(node)

    points_con = convert_points_to_name(node_dict, connections)

    for node_1, node_2, distance in points_con:
        graph.add_edge(node_1, node_2, distance)

    if time_analysis:
        print("Dijkstra Graph initializing time: {}".format(datetime.now()-start_time))

    # Initializing destinations
    if time_analysis:
        start_time = datetime.now()

    destination_node_points = np.asarray(dic.destination_node_points_town02)
    random_destination = random.choice(destination_node_points)
    start = random.choice(node_names)

    # Compute distance to random destination
    diss = []
    for pt in node_points:
        dis = compute_distance(pt, random_destination)
        diss.append([np.array(pt), dis])

    # Sorting list to find closest points
    diss = np.asarray(diss)
    diss = diss[diss[:, 1].argsort()]

    # Find shortest paths to the points that are closest to the defined random destination
    closest_points = diss[:noc, 0]

    paths = []
    for point in closest_points:
        destination = get_node_name(node_dict, point)
        dist, nodes = shortest_path(graph, start, destination)
        path = nodes_to_path(node_dict, nodes)

        # Conversion to numpy array
        path = np.asarray(path)
        paths.append([path, destination, len(path)])

    paths = np.asarray(paths)
    paths = paths[paths[:, 2].argsort()]
    path = paths[0][0]

    if time_analysis:
        print("Shortest path finding time      : {}".format(datetime.now()-start_time))

    if demo:
        if time_analysis:
            start_time = datetime.now()

        # Plotting path
        fig, ax = plt.subplots()
        ax.set_title("Path planning")
        ax.set_facecolor('gray')
        ax.scatter(node_points[:, 0], node_points[:, 1], color='white', marker='.')
        ax.plot(path[:, 0], path[:, 1], marker='.')  # linewidth=5

        # Starting of path
        ax.plot(path[0][0], path[0][1], color='green', marker="o", linewidth=10.0)

        # Destination of path
        ax.plot(path[-1][0], path[-1][1], color='red', marker="o", linewidth=10.0)

        # Random destinations on map
        ax.scatter(destination_node_points[:, 0], destination_node_points[:, 1], color='black', marker='x')

        # Chosen destination
        ax.scatter(random_destination[0], random_destination[1], color='red', marker='x')

        # Nodes selected for shortest path computation
        for pt, dis in diss[:noc]:
            ax.scatter(pt[0], pt[1], color='green', marker='o')

        # Nodes that are closest to the destination
        for pt, dis in diss[:noc+6]:
            ax.scatter(pt[0], pt[1], color='purple', marker='x', alpha=0.6)
            ax.text(pt[0] + 0.1, pt[1] + 0.1, str(round(dis, 4)))

        plt.gca().invert_yaxis()

        # Plotting nodes connections
        fig2, ax2 = plt.subplots()
        ax2.set_title("Nodes connections")
        node_points = np.asarray(node_points)
        ax2.scatter(node_points[:, 0], node_points[:, 1], color='gray', marker='.')
        for p1, p2, dis in connections:
            xs, ys = [p1[0], p2[0]], [p1[1], p2[1]]
            ax2.plot(xs, ys)

        plt.gca().invert_yaxis()

        if time_analysis:
            print("Visualizing time                : {}".format(datetime.now()-start_time))

        plt.show()

    return path


# return a predefined path: predefined start and destination
def get_course(road_segments_file, course, noc=4, demo=False, time_analysis=False):
    if time_analysis:
        print("\n== DIJKSTRA TIME ANALYSIS ===============================")
        start_time = datetime.now()

    segments = clean_road_segments(road_segments_file)
    node_points = create_nodes(segments)
    node_points = np.asarray(node_points)

    node_dict, node_names = nodes_initialization(node_points, name='t02-w')

    connections_1 = create_connection_end_nodes(segments, lower_lim=0, upper_lim=4)

    connections_2 = create_connection_segment_nodes(segments, lower_lim=0, upper_lim=2.1)
    connections = connections_1 + connections_2

    graph = Graph()

    for node in node_names:
        graph.add_node(node)

    points_con = convert_points_to_name(node_dict, connections)

    for node_1, node_2, distance in points_con:
        graph.add_edge(node_1, node_2, distance)

    if time_analysis:
        print("Dijkstra Graph initializing time: {}".format(datetime.now() - start_time))

    # Initializing destinations
    if time_analysis:
        start_time = datetime.now()

    destination_node_points = np.asarray(dic.destination_node_points_town02)
    start_idx = course[0]
    start = get_node_name(node_dict, node_points[start_idx])
    random_destination = course[1]

    # Compute distance to random destination
    diss = []
    for pt in node_points:
        dis = compute_distance(pt, random_destination)
        diss.append([np.array(pt), dis])

    # Sorting list to find closest points
    diss = np.asarray(diss)
    diss = diss[diss[:, 1].argsort()]

    # Find shortest paths to the points that are closest to the defined random destination
    closest_points = diss[:noc, 0]
    paths = []
    for point in closest_points:
        destination = get_node_name(node_dict, point)
        dist, nodes = shortest_path(graph, start, destination)
        path = nodes_to_path(node_dict, nodes)

        # Conversion to numpy array
        path = np.asarray(path)
        paths.append([path, destination, len(path)])

    paths = np.asarray(paths)
    paths = paths[paths[:, 2].argsort()]
    path = paths[0][0]

    if time_analysis:
        print("Shortest path finding time      : {}".format(datetime.now() - start_time))

    if demo:
        if time_analysis:
            start_time = datetime.now()

        # Plotting path
        fig, ax = plt.subplots()
        ax.set_title("Path planning")
        ax.set_facecolor('gray')
        ax.scatter(node_points[:, 0], node_points[:, 1], color='white', marker='.')
        ax.plot(path[:, 0], path[:, 1], marker='.')  # linewidth=5

        # Starting of path
        ax.plot(path[0][0], path[0][1], color='green', marker="o", linewidth=10.0)

        # Destination of path
        ax.plot(path[-1][0], path[-1][1], color='red', marker="o", linewidth=10.0)

        # Random destinations on map
        ax.scatter(destination_node_points[:, 0], destination_node_points[:, 1], color='black', marker='x')

        # Chosen destination
        ax.scatter(random_destination[0], random_destination[1], color='red', marker='x')

        # Nodes selected for shortest path computation
        for pt, dis in diss[:noc]:
            ax.scatter(pt[0], pt[1], color='green', marker='o')

        # Nodes that are closest to the destination
        for pt, dis in diss[:noc + 6]:
            ax.scatter(pt[0], pt[1], color='purple', marker='x', alpha=0.6)
            ax.text(pt[0] + 0.1, pt[1] + 0.1, str(round(dis, 4)))

        plt.gca().invert_yaxis()

        # Plotting nodes connections
        fig2, ax2 = plt.subplots()
        ax2.set_title("Nodes connections")
        node_points = np.asarray(node_points)
        ax2.scatter(node_points[:, 0], node_points[:, 1], color='gray', marker='.')
        for p1, p2, dis in connections:
            xs, ys = [p1[0], p2[0]], [p1[1], p2[1]]
            ax2.plot(xs, ys)

        plt.gca().invert_yaxis()

        if time_analysis:
            print("Visualizing time                : {}".format(datetime.now() - start_time))

        plt.show()

    return path
