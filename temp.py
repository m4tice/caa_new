import caa_new.my_utils.tools as to
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

my_list = np.arange(0, 1000000, 1)
my_list_2 = []

start_time = datetime.now()

@asyncio
for item in my_list:
    my_list_2.append(item+1)

print("Processing time:", datetime.now()-start_time)