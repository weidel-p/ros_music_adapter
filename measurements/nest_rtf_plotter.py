import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

from plotcolors import *
import seaborn as sbn

sbn.set_palette("Blues", desat=0.)
sbn.set_context("talk", font_scale = 4., rc={"figure.figsize": (32, 16), "lines.linewidth": 8.})
sbn.set_style("ticks", {'axes.linewidth': 6.0, 'xtick.major.size': 16.0, 'ytick.major.size': 16.0,'xtick.minor.size': 12.0, 'ytick.minor.size': 12.0})
#palette = [(0.0, 0.0, 0.0), (0.66, 0.66, 0.66), (0.33, 0.33, 0.33)]
#palette = [(0.0, 0.0, 0.0), (200./255., 20./255., 20./255.), (70./255., 70./255., 232./255.)]
palette = [myblack, myred, myblue]


data_file = open(sys.argv[1], 'r')

data = pd.DataFrame(json.load(data_file))
data_file.close()


#plot data
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data, color=palette)
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")
ax.get_legend().set_title("")
ax.set_ylim([0.6, 1.05])
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
#ax.get_legend().set_title("")
#ax.legend([])

plt.savefig("nest_rtf.pdf")
plt.show()

