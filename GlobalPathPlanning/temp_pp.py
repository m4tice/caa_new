from caa_new.my_utils import dictionaries as dic
from caa_new.my_utils import tools as to
from caa_new.GlobalPathPlanning.module_p2 import get_random_course
from caa_new.GlobalPathPlanning.module_p2 import *

town_dic = dic.town02
starts = [2066, 1766, 355, 649]

map_name,\
    weather_type,\
    current_town,\
    img_dir,\
    csv_file,\
    lidar_dir,\
    gnss_csv,\
    gps_intersection_csv,\
    waypoints_csv,\
    spawn_csv,\
    carla_intersection_csv,\
    road_segments_file = to.paths_initialization(town_dic)

path = get_course(road_segments_file, dic.course_02, demo=True, time_analysis=True)

plt.gca().invert_yaxis()
plt.show()
