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


#plot data
plt.title("Bandwidth")
ax = sbn.tsplot(time="firing_rate", value="time", unit="iteration", condition="type", data=data)
ax.set_ylabel("real-time factor")
ax.set_xlabel("firing-rate")

plt.savefig("bandwidth.pdf")
plt.show()

