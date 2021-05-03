import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import csv
from caa.c_utils import module_p2 as mp2
road_segments_file = "../database/town04/town04_road_segments.txt"

segments = mp2.clean_road_segments(road_segments_file)

fig, ax = plt.subplots()
ax.set_title("Town04 map")

for idx, seg in enumerate(segments):
    if len(seg[0]) == 0 or len(seg[1]) == 0:
        pass
    else:
        ax.plot(seg[0], seg[1], color='blue', label='_nolegend_')
        # ax.text(seg[0][0], seg[1][0], str(idx))


def load_csv_data(csv_file):
    data_df = pd.read_csv(csv_file, names=['x', 'y'])
    data = data_df[['x', 'y']].values

    return data

# path1_id = [149, 19, 178, 243, 69]
# path2_id = [150, 20, 179, 242, 70]
#
# path1_x, path2_x = [], []
# path1_y, path2_y = [], []
#
#
# for id1, id2 in zip(path1_id, path2_id):
#     path1_x.extend(segments[id1][0])
#     path1_y.extend(segments[id1][1])
#     path2_x.extend(segments[id2][0])
#     path2_y.extend(segments[id2][1])


path1 = load_csv_data("course1.csv")
path2 = load_csv_data("course2.csv")

path1 = np.asarray(path1)
path2 = np.asarray(path2)

path1_x, path1_y = path1[:, 0], path1[:, 1]
path2_x, path2_y = path2[:, 0], path2[:, 1]

ax.plot(path1_x, path1_y, color='red', )
ax.plot(path2_x, path2_y, color='green')

ax.legend(["course 1", "course 2"])
plt.gca().invert_yaxis()

# with open("course1.csv", 'w', newline='') as file:
#     writer = csv.writer(file)
#     for loc_x, loc_y in zip(path1_x, path1_y):
#         writer.writerow([loc_x, loc_y])
#
# with open("course2.csv", 'w', newline='') as file:
#     writer = csv.writer(file)
#     for loc_x, loc_y in zip(path2_x, path2_y):
#         writer.writerow([loc_x, loc_y])

plt.show()
