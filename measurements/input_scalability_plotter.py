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


# split data
rate_rtf_ind = [i for i in range(len(data_rate["type"])) if data_rate["type"][i] == "real-time factor"]
rate_ind = [i for i in range(len(data_rate["type"])) if not data_rate["type"][i] == "real-time factor"]
data_rate_rtf = data_rate.drop(data_rate.index[rate_ind])
data_rate = data_rate.drop(data_rate.index[rate_rtf_ind])

poisson_rtf_ind = [i for i in range(len(data_poisson["type"])) if data_poisson["type"][i] == "real-time factor"]
poisson_ind = [i for i in range(len(data_poisson["type"])) if not data_poisson["type"][i] == "real-time factor"]
data_poisson_rtf = data_poisson.drop(data_poisson.index[poisson_ind])
data_poisson = data_poisson.drop(data_poisson.index[poisson_rtf_ind])

nef_rtf_ind = [i for i in range(len(data_nef["type"])) if data_nef["type"][i] == "real-time factor"]
nef_ind = [i for i in range(len(data_nef["type"])) if not data_nef["type"][i] == "real-time factor"]
data_nef_rtf = data_nef.drop(data_nef.index[nef_ind])
data_nef = data_nef.drop(data_nef.index[nef_rtf_ind])

#plot data
ax = plt.subplot(2,3,1)
ax.set_title("Rate Encoder")
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_rate)
ax.set_ylabel("time [s]")
ax.set_xlabel("# neurons")
ax = plt.subplot(2,3,4)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_rate_rtf, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")

ax = plt.subplot(2,3,2)
ax.set_title("Poisson Encoder")
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_poisson)
ax.set_ylabel("time [s]")
ax.set_xlabel("# neurons")
ax = plt.subplot(2,3,5)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_poisson_rtf, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")

ax = plt.subplot(2,3,3)
ax.set_title("NEF Encoder")
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_nef)
ax.set_ylabel("time [s]")
ax.set_xlabel("# neurons")
ax = plt.subplot(2,3,6)
ax = sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data_nef_rtf, color=palette[2:])
ax.set_ylabel("real-time factor")
ax.set_xlabel("# neurons")


plt.savefig("input_scalability.pdf")
plt.show()

