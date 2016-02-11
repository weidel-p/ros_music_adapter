import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

from plotcolors import *
import seaborn as sbn
sbn.set_palette("Blues", desat=0.)
sbn.set_context("talk", font_scale = 4., rc={"figure.figsize": (32, 16), "lines.linewidth": 12.})
sbn.set_style("ticks", {'axes.linewidth': 8.0, 'xtick.major.size': 24.0, 'ytick.major.size': 24.0,'xtick.minor.size': 18.0, 'ytick.minor.size': 18.0})
palette = [myblack, myred, myblue]

with open(sys.argv[1], 'r') as data:
    data_bandwidth = pd.DataFrame(json.load(data))

with open(sys.argv[2], 'r') as data:
    data_latency = pd.DataFrame(json.load(data))


#plot data
plt.subplot(1,2,1)
ax = sbn.tsplot(time="firing_rate", value="time", unit="iteration", condition="type", data=data_bandwidth, color=palette)
ax.set_ylabel("real-time factor")
ax.set_xlabel("firing rate (spikes/sec)")
ax.get_legend().set_title("")
ax.set_ylim([0.6, 1.05])
ax.get_legend().set_title("")
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
ax.legend([])
ax.text(-0.1, 1.02, "a", transform=ax.transAxes, size=72, weight='bold')

plt.subplot(1,2,2)
ax = sbn.tsplot(time="timestep", value="latency", unit="iteration", condition="type", data=data_latency, color=palette)
sbn.axlabel("MUSIC time step (s)", "latency (s)")
ax.get_legend().set_title("")
ax.legend([])
ax.set_ylim([0., 0.34])
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
ax.text(-0.1, 1.02, "b", transform=ax.transAxes, size=72, weight='bold')

plt.subplots_adjust(bottom=0.14, top=0.93, left=0.07, right=0.97, wspace=0.25)

plt.savefig("bandwidth_latency.pdf", dpi=500)
#plt.show()

