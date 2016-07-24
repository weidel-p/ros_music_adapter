import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

from plotcolors import *
import seaborn as sbn

sbn.set_palette("Blues", desat=0.)
sbn.set_context("talk", font_scale = 3., rc={"figure.figsize": (32, 16), "lines.linewidth": 8., 'lines.markeredgewidth': 1.5 })
sbn.set_style("ticks", {'axes.linewidth': 6.0, 'xtick.major.size': 16.0, 'ytick.major.size': 16.0,'xtick.minor.size': 12.0, 'ytick.minor.size': 12.0, "ytick.direction": "in", "xtick.direction": "in"})
#palette = [(0.0, 0.0, 0.0), (0.66, 0.66, 0.66), (0.33, 0.33, 0.33)]
#palette = [(0.0, 0.0, 0.0), (200./255., 20./255., 20./255.), (70./255., 70./255., 232./255.)]
palette = [myblack, myred, myblue]


data_file = open(sys.argv[1], 'r')

data = pd.DataFrame(json.load(data_file))
data_file.close()

#plot data

ax = plt.subplot(1,2,2)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", ci = 68, data=data, color=palette)
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")
ax.get_legend().set_title("")
ax.set_ylim([0.6, 1.05])
ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')

ax.get_figure().subplots_adjust(bottom=0.12)
ax.get_legend().set_title("")
ax.legend([])
ax.text(-0.1, 1.02, "b", transform=ax.transAxes, size=72, weight='bold')


spike_data = np.loadtxt(sys.argv[2])
senders = spike_data[:,0] 
senders -= min(senders)
times = spike_data[:,1]

mask = np.where(np.logical_and(np.logical_and(times < 1200, times > 200), senders < 1000) )
senders = senders[mask]
times = times[mask]
times -= min(times)


#if ax is None:
#    if with_rate:
#        ax = pl.subplot2grid((30, 1), (0, 0), rowspan=23, colspan=1)
#        ax.set_xticklabels([])
#        ax2 = pl.subplot2grid((30, 1), (24, 0), rowspan=5, colspan=1)
#        ax2.set(xlabel='Time [ms]', ylabel='Rate')
#        ax.set(ylabel='Neuron')
#    else:
#        ax = fig.add_subplot(111)
#else:
#    if with_rate:
#        assert isinstance(ax, list), "Incompatible properties... (with_rate requires two axes provided or " \
#                                     "None)"
#        ax = ax[0]
#        ax2 = ax[1]

ax = plt.subplot2grid((30, 2), (0, 0), rowspan=24, colspan=1)
ax2 = plt.subplot2grid((30, 2), (25, 0), rowspan=5, colspan=1)
ax.plot(times, senders, '|', color=palette[0])
ax.set_ylabel("neuron")
ax.set_xticklabels([])
ax.text(-0.2, 1.02, "a", transform=ax.transAxes, size=72, weight='bold')

ax2.set(xlabel='time (ms)', ylabel='rate (Hz)')

b, v = np.histogram(times, 100)
#b /= 1000.

ax2.plot(v[:-1], b/10., color=palette[0], linewidth=1.5)
ax2.locator_params(axis='y',nbins=5)

#if with_rate:
#    time = self.time_axis(dt)[:-1]
#    rate = self.firing_rate(dt, average=True)
#    if not times == None:
#        ax2.set(ylim=[0., max(rate[times[0]:times[1]])], xlim=times)
#    else:
#        ax2.set(ylim=[0., max(rate)*1.5], xlim=[self.t_start, self.t_stop])
#    ax2.plot(time, rate, **kwargs)
#
#ax.set(ylim=[min(self.id_list), max(self.id_list)], xlim=[self.t_start, self.t_stop])
plt.subplots_adjust(0.09, 0.12, 0.96, 0.9, 0.25, 0.1)


plt.savefig(sys.argv[1] + ".pdf")
#plt.show()

