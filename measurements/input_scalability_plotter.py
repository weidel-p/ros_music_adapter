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

# load files
data_file_rate = open(sys.argv[1], 'r')
data_file_poisson = open(sys.argv[2], 'r')
data_file_nef = open(sys.argv[3], 'r')

data_rate = pd.DataFrame(json.load(data_file_rate))
data_poisson = pd.DataFrame(json.load(data_file_poisson))
data_nef = pd.DataFrame(json.load(data_file_nef))

data_file_rate.close()
data_file_poisson.close()
data_file_nef.close()

x = plt.subplot(1,3,1)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_rate, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 3000000])
ax.set_ylim([0.6, 1.05])
ax.get_legend().set_title("")
ax.legend([])
ax.text(-0.1, 1.02, "a", transform=ax.transAxes, size=52, weight='bold')

ax = plt.subplot(1,3,2)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_poisson, color=palette[2:])
ax.set_ylabel("")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 3000000])
ax.set_ylim([0.6, 1.05])
ax.get_legend().set_title("")
ax.legend([])
ax.text(-0.1, 1.02, "b", transform=ax.transAxes, size=52, weight='bold')

ax = plt.subplot(1,3,3)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_nef, color=palette[2:])
ax.set_ylabel("")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 3000000])
ax.set_ylim([0.6, 1.05])
ax.get_legend().set_title("")
#ax.legend(fontsize=36)
ax.legend([])
ax.text(-0.1, 1.02, "c", transform=ax.transAxes, size=52, weight='bold')


plt.savefig("input_scalability.pdf")
plt.show()

