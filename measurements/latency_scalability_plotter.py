import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

import seaborn as sbn
sbn.set_palette("deep", desat=.6)
sbn.set_context(rc={"figure.figsize": (16, 8)})
sbn.set_style("whitegrid")
sbn.despine()
palette = sbn.color_palette()

data_file = open(sys.argv[1], 'r')
data = pd.DataFrame(json.load(data_file))
data_file.close()

#drop data about other types then rtf
data.drop([i for i in range(len(data["type"])) if data["type"][i] == "total-time" or data["type"][i] == "build-time"], inplace=True)

# drop information about the iteration and type
data.drop("iteration", axis=1, inplace=True)
data.drop("type", axis=1, inplace=True)

print data

data_grouped = data.groupby(["timestep", "num_neurons"])

rtfs = np.array([])
for name, group in data_grouped:
    rtfs = np.append(rtfs, group.mean()["time"])

x = np.unique(data["num_neurons"])
y = np.unique(data["timestep"])[::-1]
rtfs = np.reshape(rtfs, [len(y), len(x)])

ax = sbn.heatmap(rtfs)
ax.set_xticklabels(x)
ax.set_yticklabels(y)
plt.show()


