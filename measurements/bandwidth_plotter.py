import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np

import seaborn as sbn
sbn.set_palette("deep", desat=.6)
sbn.set_context(rc={"figure.figsize": (8, 4)})
palette = sbn.color_palette()

data_file = open("bandwidth.dat", 'r')

data = pd.DataFrame(json.load(data_file))
data_file.close()

sbn.pointplot(x="firing_rate", y="run_time", data=data, color=palette[0])
sbn.pointplot(x="firing_rate", y="build_time", data=data, color=palette[1])
sbn.pointplot(x="firing_rate", y="real-time_factor", data=data, color=palette[2])
plt.show()

