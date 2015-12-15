import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys
import colormaps as cmaps


import seaborn as sbn
sbn.set_palette("deep", desat=.6)
sbn.set_context(rc={"figure.figsize": (16, 8)})
sbn.set_style("whitegrid")
sbn.despine()
palette = sbn.color_palette()

data_file = open(sys.argv[1], 'r')
data = pd.DataFrame(json.load(data_file))
data_file.close()

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

cmap = plt.get_cmap("copper")

ax = sbn.heatmap(rtfs, linewidth=0., cmap=cmap)
#ax.set_xticks(np.linspace(0, 20000, 10))
ax.set_xticklabels(x)
for label in ax.get_xticklabels()[::2]:
    label.set_visible(False)
ax.set_yticklabels(y)
ax.set_xlabel("#neurons")
ax.set_ylabel("MUSIC-timestep [s]")
plt.title("MUSIC-Timestep vs Scalability")
plt.savefig("latency_scalability.pdf")
plt.show()


