import matplotlib.pyplot as plt
import json
import pandas as pd
import numpy as np
import sys

import seaborn as sbn
sbn.set_palette("deep", desat=.6)
sbn.set_context(rc={"figure.figsize": (8, 4)})
palette = sbn.color_palette()

data_file = open(sys.argv[1], 'r')

data = pd.DataFrame(json.load(data_file))
data_file.close()

print data

#sbn.tsplot(time="num_neurons", value="real-time factor", unit="iteration", condition="num_threads", data=data)
plt.plot(data["real-time factor"])
plt.plot(data["num_threads"])
plt.plot(np.array(data["num_neurons"]) / 1000.)
plt.savefig("nest_scalability.pdf")
plt.show()
