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


ax = sbn.tsplot(time="timestep", value="latency", unit="iteration", condition="type", data=data, color=palette[2:])
#sbn.boxplot(x="timestep", y="latency", data=data)
sbn.axlabel("MUSIC timestep (s)", "latency (s)")
#sbn.tsplot(time="times", value="cmds", data=data)

ax.get_legend().set_title("")
ax.legend([])
plt.savefig("latency.pdf", dpi=500)

plt.show()
