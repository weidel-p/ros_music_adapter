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

# split data
rtf_ind = [i for i in range(len(data["type"])) if data["type"][i] == "real-time factor"]
ind = [i for i in range(len(data["type"])) if not data["type"][i] == "real-time factor"]
data_rtf = data.drop(data.index[ind])
data = data.drop(data.index[rtf_ind])


#plot data
ax = plt.subplot(1,2,1)
plt.title("Bandwidth")
ax = sbn.tsplot(time="firing_rate", value="time", unit="iteration", condition="type", data=data)
ax.set_ylabel("time [s]")
ax.set_xlabel("firing-rate")
ax = plt.subplot(1,2,2)
plt.title("Real-time factor")
ax = sbn.tsplot(time="firing_rate", value="time", unit="iteration", condition="type", data=data_rtf, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("firing-rate")

plt.savefig("bandwidth.pdf")
plt.show()

