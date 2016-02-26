import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys
import colormaps as cmaps

import seaborn as sbn
sbn.set_palette("Blues", desat=0.)
sbn.set_context("talk", font_scale = 4., rc={"figure.figsize": (32, 16), "lines.linewidth": 8.})
sbn.set_style("ticks", {'axes.linewidth': 6.0, 'xtick.major.size': 16.0, 'ytick.major.size': 16.0,'xtick.minor.size': 12.0, 'ytick.minor.size': 12.0})
#palette = np.array(sbn.color_palette())[[5,3,2]]
#palette = [(0.0, 0.0, 0.0), (0.66, 0.66, 0.66), (0.33, 0.33, 0.33)]
#palette = [(0.0, 0.0, 0.0), (200./255., 20./255., 20./255.), (70./255., 70./255., 232./255.)]


with open(sys.argv[1], 'r') as data_file:
    data = pd.DataFrame(json.load(data_file))

# drop information about the iteration and type
data.drop("iteration", axis=1, inplace=True)
data.drop("type", axis=1, inplace=True)

data_grouped = data.groupby(["timestep", "num_neurons"])

rtfs = np.array([])
for name, group in data_grouped:
    rtfs = np.append(rtfs, group.mean()["time"])

x = np.unique(data["num_neurons"])
y = np.unique(data["timestep"])[::-1]
y = y * 1000.
y = y.astype(int)

rtfs = np.reshape(rtfs, [len(y), len(x)])

cmap = plt.get_cmap("copper")

plt.imshow(rtfs, cmap=cmap, aspect='auto')
ax = plt.axes()
ax.set_xticks(range(len(x)))
ax.set_yticks(range(len(y)))

labels = ax.get_xticklabels()
xticks = ax.get_xticks()
for i in range(len(labels)):
    label = labels[i]
    xtick = xticks[i]
    if i % 5 == 1:
        label.set_visible(True)
    else:
        label.set_visible(False)
        xticks[i] = xticks[1]

ax.set_xticklabels(x)
ax.set_yticklabels(y[::-1])
ax.set_xlabel("#neurons")
ax.set_ylabel("MUSIC time step (ms)")
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
cbar = plt.colorbar()
cbar.set_label('real-time factor', rotation=270)
cbar.ax.get_yaxis().labelpad = 65

plt.subplots_adjust(bottom=0.14, top=0.95, left=0.10, right=1.03)
plt.savefig("latency_scalability.pdf")


