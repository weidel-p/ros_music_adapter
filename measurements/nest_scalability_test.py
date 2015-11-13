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
	



