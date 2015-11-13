import numpy as np
import sys
import os
import nest
import json
from datetime import datetime as time

def run(SIMTIME=1., NUM_NEURONS=1, NUM_THREADS=1):
	nest.ResetKernel()
	nest.SetKernelStatus({'local_num_threads': NUM_THREADS, 'total_num_virtual_procs': NUM_THREADS, 'print_time': False})
        nest.SetKernelStatus({'resolution': 1.0})
	nest.set_verbosity("M_ERROR")
	
	neurons = nest.Create("iaf_neuron", NUM_NEURONS, {"I_e": 375.0001})
	nest.Connect(neurons, neurons, "all_to_all", {"weight": 0.})
	
	sd = nest.Create("spike_detector")
	nest.Connect(neurons, sd)
	
	nest.Simulate(SIMTIME)
	
	events = nest.GetStatus(sd, 'events')[0]
	
	rate = len(events['times']) / (float(NUM_NEURONS) * SIMTIME/1000.)
	


data_filename = sys.argv[1] 

if os.path.exists(data_filename):
    os.remove(data_filename)



data = {"num_neurons": [], "num_threads": [], "iteration": [], "real-time factor": []}
T_BUILD = 1
T_TOTAL = 1000
NUM_THREADS = 1
NUM_NEURONS = 100

while NUM_THREADS <= 5:
	rtf = [] 

	for i in range(3):
		start_build = time.now()
		run(T_BUILD, NUM_NEURONS, NUM_THREADS)
		end_build = time.now()
		dt_build = end_build - start_build
		build_time = dt_build.seconds + dt_build.microseconds / 1000000. 
		
		start_total = time.now()
		run(T_TOTAL, NUM_NEURONS, NUM_THREADS)
		end_total = time.now()
		dt_total = end_total - start_total
		total_time = dt_total.seconds + dt_total.microseconds / 1000000. 

		_rtf = (T_TOTAL / 1000.) / (total_time - build_time)
		data["num_neurons"].append(NUM_NEURONS)
		data["num_threads"].append(NUM_THREADS)
		data["iteration"].append(i)
		data["real-time factor"].append(_rtf)
		rtf.append( _rtf )

	rtf = np.mean(rtf)
	print NUM_THREADS, NUM_NEURONS, rtf

	if rtf < 1:
		NUM_THREADS += 1
	else:
		NUM_NEURONS += 100
	
data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()

