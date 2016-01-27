import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

import seaborn as sbn
sbn.set_palette("deep", desat=.6)
sbn.set_context("talk", font_scale = 4., rc={"figure.figsize": (32, 16), "lines.linewidth": 8.})
sbn.set_style("white")
sbn.despine()
palette = sbn.color_palette()


data_file = open(sys.argv[1], 'r')

data = pd.DataFrame(json.load(data_file))
data_file.close()


#plot data
ax = sbn.tsplot(time="firing_rate", value="time", unit="iteration", condition="type", data=data, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("firing-rate (spikes/sec)")
ax.get_legend().set_title("")
ax.set_ylim([0.6, 1.05])
ax.get_legend().set_title("")
ax.legend([])

plt.savefig("bandwidth.pdf")
plt.show()

