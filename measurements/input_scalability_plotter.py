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

ax = plt.subplot(1,3,1)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_rate, color=palette, err_style=None)
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 1200000])
ax.set_ylim([0.6, 1.05])
ax.tick_params(axis='x', pad=15)
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
ax.get_legend().set_title("")
ax.legend([])
ax.text(-0.1, 1.02, "a", transform=ax.transAxes, size=72, weight='bold')

ax = plt.subplot(1,3,2)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_poisson, color=palette, err_style=None)
ax.set_ylabel("")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 1200000])
ax.set_ylim([0.6, 1.05])
ax.tick_params(axis='x', pad=15)
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
ax.set_yticklabels([])
ax.get_legend().set_title("")
ax.legend([])
ax.text(-0.1, 1.02, "b", transform=ax.transAxes, size=72, weight='bold')

ax = plt.subplot(1,3,3)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_nef, color=palette, err_style=None)
ax.set_ylabel("")
ax.set_xlabel("# neurons")
ax.set_xscale('log')
ax.set_xlim([100, 1200000])
ax.set_ylim([0.6, 1.05])
ax.tick_params(axis='x', pad=15)
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
ax.set_yticklabels([])
ax.get_legend().set_title("")
#ax.legend(fontsize=36)
ax.legend([])
ax.text(-0.1, 1.02, "c", transform=ax.transAxes, size=72, weight='bold')

plt.subplots_adjust(bottom=0.14, top=0.93, left=0.07, right=0.97)

plt.savefig("input_scalability.pdf")
#plt.show()

