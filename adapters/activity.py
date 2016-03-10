#!/usr/bin/python

import music
from mpi4py import MPI
from matplotlib import mlab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

plt.ion()
class DynamicUpdate():
    #Suppose we know the x range

    ax_min = -20
    ax_max = 20

    def on_launch(self):
        #Set up plot
        #self.figure, self.ax = plt.subplots()

        self.figure = plt.figure("Neural Activity")
        self.ax0 = self.figure.add_subplot(111)
        self.ax0.set_xlabel("time [s]")
        self.ax0.set_ylabel("neuron id")
        self.ax0.set_ylim(-0.5, 1.5)

        self.lines, = self.ax0.plot([],[], 'ok')
        #Autoscale on unknown axis and known lims on the other
#        self.ax.set_autoscaley_on(True)
        #Other stuff
        self.ax0.grid()

    def on_running(self, xdata, ydata):
        self.ax0.set_xlim(min(xdata), max(xdata))
        #self.ax.set_zlim(self.ax_min, self.ax_max)
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)

       #Need both of these in order to rescale
        self.ax0.relim()
        self.ax0.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


comm = MPI.COMM_WORLD

global DEFAULT_TIMESTEP, timestep, setup, runtime, stoptime, spikes

DEFAULT_TIMESTEP = 0.001
SPIKE_HIST_LENGTH = 0.3 # seconds

spikes = {'times': np.array([0]), 'senders': np.array([0])}

def main():
    init()
    initMUSIC()
    runMUSIC()

def init():
    print("initializing activity adapter")

def eventfunc(d, t, i):
    global port_out, spikes
    #print "inc spike", i, d, t

    spikes['times'] = np.append(d, spikes['times'])
    spikes['senders']= np.append(i, spikes['senders'])
    port_out.insertEvent(d, i, music.Index.GLOBAL)

def initMUSIC():
    global DEFAULT_TIMESTEP, timestep, setup, stoptime, runtime, d, num_neurons, port_out

    setup = music.Setup()
    try:
        timestep = setup.config("music_timestep")
    except:
        timestep = DEFAULT_TIMESTEP

    stoptime = setup.config("stoptime")

    port_in = setup.publishEventInput("in")

    port_in.map(eventfunc, 
        music.Index.GLOBAL, 
        base=0, 
        size=port_in.width(),
        maxBuffered=1)

    port_out = setup.publishEventOutput("out")
   
    port_out.map(music.Index.GLOBAL,
        base=0,
        size=port_out.width(),
        maxBuffered=1) 

    d = DynamicUpdate()
    d.on_launch()

    num_neurons = port_in.width()

    comm.Barrier()
    runtime = music.Runtime(setup, timestep)

def runMUSIC():
    global runtime, stoptime, timestep, state, state_hist, d, PCA_HIST_LENGTH, spikes, SPIKE_HIST_LENGTH, proj_hist
    print "running activity adapter"
    t = 0 
    
    while runtime.time() < stoptime:
        if t % 20 == 0:
            spike_hist_mask = np.where(spikes['times'] > max(spikes['times']) - SPIKE_HIST_LENGTH)
            spikes['times'] = spikes['times'][spike_hist_mask]
            spikes['senders'] = spikes['senders'][spike_hist_mask]

            d.on_running(spikes['times'], spikes['senders'])

        runtime.tick()
        t += 1 


if __name__ == "__main__":
    main()
