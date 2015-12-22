import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys
import colormaps as cmaps


#import seaborn as sbn
#sbn.set_palette("deep", desat=.6)
#sbn.set_context(rc={"figure.figsize": (16, 8), "linewidths": 0.})
#sbn.set_style("whitegrid")
#sbn.despine()
#palette = sbn.color_palette()

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

print x, y

fig = plt.figure(1, figsize=(20, 10))

cmap = plt.get_cmap("copper")

plt.imshow(rtfs, cmap=cmap, interpolation="None", aspect='auto')
ax = plt.axes()
ax.set_xticks(range(len(x)))
ax.set_yticks(range(len(y)))

ax.set_xticklabels(x, size=10.)
for label in ax.get_xticklabels()[::2]:
    label.set_visible(False)
ax.set_yticklabels(y[::-1], size=10.)
ax.set_xlabel("#neurons", size=20.)
ax.set_ylabel("MUSIC-timestep [s]", size=20.)
plt.title("Real-time factor", size=20.)
plt.colorbar(title="blub")

plt.savefig("latency_scalability.pdf")
plt.show()


