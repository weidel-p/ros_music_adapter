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

sbn.tsplot(time="num_neurons", value="time", unit="iteration", condition="type", data=data)
plt.savefig("input_scalability.pdf", dpi=500)
plt.show()

