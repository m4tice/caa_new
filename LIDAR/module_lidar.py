import numpy as np
import math


# A Python3 program to check if a given point lies inside a given polygon
# Refer https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
# for explanation of functions onSegment(), orientation() and doIntersect()

# Define Infinite (Using INT_MAX caused overflow problems)
INT_MAX = 10000


# Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
def onSegment(p: tuple, q: tuple, r: tuple) -> bool:
    if ((q[0] <= max(p[0], r[0])) &
            (q[0] >= min(p[0], r[0])) &
            (q[1] <= max(p[1], r[1])) &
            (q[1] >= min(p[1], r[1]))):
        return True

    return False


# To find orientation of ordered triplet (p, q, r).
# The function returns following values
# 0 --> p, q and r are colinear
# 1 --> Clockwise
# 2 --> Counterclockwise
def orientation(p: tuple, q: tuple, r: tuple) -> int:
    val = (((q[1] - p[1]) *
            (r[0] - q[0])) -
           ((q[0] - p[0]) *
            (r[1] - q[1])))

    if val == 0:
        return 0
    if val > 0:
        return 1  # Collinear
    else:
        return 2  # Clock or counterclock


def doIntersect(p1, q1, p2, q2):
    # Find the four orientations needed for general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0) and (onSegment(p1, p2, q1)):
        return True

    # p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0) and (onSegment(p1, q2, q1)):
        return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0) and (onSegment(p2, p1, q2)):
        return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0) and (onSegment(p2, q1, q2)):
        return True

    return False


# Returns true if the point p lies inside the polygon[] with n vertices
def is_inside_polygon(points: list, p: tuple) -> bool:
    n = len(points)

    # There must be at least 3 vertices in polygon
    if n < 3:
        return False

    # Create a point for line segment from p to infinite
    extreme = (INT_MAX, p[1])
    count = i = 0

    while True:
        next = (i + 1) % n

        # Check if the line segment from 'p' to 'extreme' intersects with the line
        # segment from 'polygon[i]' to 'polygon[next]'
        if doIntersect(points[i], points[next], p, extreme):

            # If the point 'p' is colinear with line segment 'i-next', then check if it lies on segment.
            # If it lies, return true, otherwise false
            if orientation(points[i], p, points[next]) == 0:
                return onSegment(points[i], p, points[next])

            count += 1

        i = next

        if i == 0:
            break

    # Return true if count is odd, false otherwise
    return count % 2 == 1


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


def lidar_detection(point_cloud, vehicle, path, cys):
    bx = 2.396
    by = 1.082
    yd = 0.1
    tr = [bx, by]
    tl = [-bx, by]
    br = [-bx, -by]
    bl = [bx, -by]
    bb = [tr, tl, br, bl]

    try:
        # point cloud
        points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        points = strip_data(points, 1, -1.5)
        points = np.array([item for item in points if not is_inside_polygon(bb, (item[0], item[1]))])

        # get location information
        location = vehicle.get_location()
        loc_x, loc_y = location.x, location.y

        # get transform information
        degree = vehicle.get_transform().rotation.yaw
        theta = degree * np.pi / 180

        Tx, Ty = transform_distance([0, 0], [loc_x, loc_y])
        org = translate([loc_x, loc_y], Tx, Ty)

        path = np.asarray([translate(item, Tx, Ty) for item in path])
        path = np.asarray([rotate(item[0], item[1], org[0], org[1], -theta) for item in path])

        p1, p2 = [], []
        for p, cy in zip(path, cys):
            temp_line = np.array([[p[0], p[1] + by + yd], [p[0], p[1] - by - yd]])
            temp_line = np.array([rotate(item[0], item[1], p[0], p[1], cy) for item in temp_line])
            temp_line = np.array([rotate(item[0], item[1], p[0], p[1], -theta) for item in temp_line])
            p1.append(temp_line[0])
            p2.append(temp_line[1])

        p2.reverse()
        p1.extend(p2)
        p2 = [[0, 0 + by + yd], [bx * 2, 0 + by + yd], [bx * 2, 0 - by - yd], [0, 0 - by - yd]]

        shorten_points = np.array([point for point in points if point[0] > 0])

        in_region1 = [[point[0], point[1]] for point in shorten_points if is_inside_polygon(p1, (point[0], point[1]))]
        in_region2 = [[point[0], point[1]] for point in shorten_points if is_inside_polygon(p2, (point[0], point[1]))]

        if len(in_region1) > 0 or len(in_region2) > 0:
            print("Lidar module: Obstacle detected!")
            return 1
        else:
            return 0

    except Exception as e:
        pass
