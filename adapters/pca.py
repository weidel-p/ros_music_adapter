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

        self.figure = plt.figure()
        self.ax0 = self.figure.add_subplot(111)
        self.ax1 = self.figure.add_subplot(122, projection = '3d')

        self.lines, = self.ax0.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
#        self.ax.set_autoscaley_on(True)
        #Other stuff
        self.ax0.grid()

    def on_running(self, xdata, ydata, pca):
        self.ax0.set_xlim(min(xdata), max(xdata))
        #self.ax.set_zlim(self.ax_min, self.ax_max)
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)

        self.ax1.clear()

        self.ax1.set_xlim(self.ax_min, self.ax_max)
        self.ax1.set_ylim(self.ax_min, self.ax_max)
        self.ax1.set_ylim(self.ax_min, self.ax_max)
        try:
            self.ax1.plot(pca[:,0], pca[:,1], pca[:,2])
        except:
            pass
        #Need both of these in order to rescale
        self.ax0.relim()
        self.ax0.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


comm = MPI.COMM_WORLD

global DEFAULT_TIMESTEP, timestep, setup, runtime, stoptime, tau, DEFAULT_TAU, state, state_hist, PCA_HIST_LENGTH, PROJ_HIST_LENGTH, spikes

DEFAULT_TIMESTEP = 0.001
DEFAULT_TAU = 0.18
PCA_HIST_LENGTH = 20 # in sec
SPIKE_HIST_LENGTH = 1 # seconds
PROJ_HIST_LENGTH = 5 #in sec
spikes = {'times': np.array([0]), 'senders': np.array([0])}

def main():
    init()
    initMUSIC()
    runMUSIC()

def init():
    print("initializing PCA adapter")

def eventfunc(d, t, i):
    global state, tau, spikes
    state[i] += tau

    spikes['times'] = np.append(d, spikes['times'])
    spikes['senders']= np.append(i, spikes['senders'])
    #print "inc spike", i, d, t


def initMUSIC():
    global DEFAULT_TIMESTEP, timestep, setup, stoptime, runtime, tau, DEFAULT_TAU, state, state_hist, d, num_neurons, proj_hist

    setup = music.Setup()
    try:
        timestep = setup.config("music_timestep")
    except:
        timestep = DEFAULT_TIMESTEP

    try:
        tau = setup.config("tau")
    except:
        tau = DEFAULT_TAU

    stoptime = setup.config("stoptime")

    port_in = setup.publishEventInput("in")

    port_in.map(eventfunc, 
        music.Index.GLOBAL, 
        base=0, 
        size=port_in.width(),
        maxBuffered=1)

    state = np.ones(port_in.width())
    state_hist = {"states": [np.array(state)], "times": [0]}
    for i in range(100):
        state_hist['states'] = np.append(state_hist['states'], [state], axis = 0)
        state_hist['times'] = np.append(state_hist['times'], [0], axis = 0)

    proj = np.zeros(3)
    proj_hist = {"projs": [np.array(proj)], "times": [0]}

    d = DynamicUpdate()
    d.on_launch()

    num_neurons = port_in.width()

    comm.Barrier()
    runtime = music.Runtime(setup, timestep)

def runMUSIC():
    global runtime, stoptime, timestep, state, state_hist, d, PCA_HIST_LENGTH, spikes, SPIKE_HIST_LENGTH, proj_hist
    print "running PCA adapter"
    t = 0 
    pca_created = False
    
    while runtime.time() < stoptime:
        
        if runtime.time() > PCA_HIST_LENGTH and not pca_created:
            pca = mlab.PCA(state_hist['states'])
            pca_created = True

        state = state * np.exp(-timestep/ tau)
        if t % 20 == 0:
            if runtime.time() < PCA_HIST_LENGTH:
                state_hist['states'] = np.append(state_hist['states'], [state], axis = 0)
                state_hist['times'] = np.append(state_hist['times'], [runtime.time()], axis = 0)

                state_hist_mask = np.where(state_hist['times'] > max(state_hist['times']) - PCA_HIST_LENGTH)
                state_hist['times'] = state_hist['times'][state_hist_mask]
                state_hist['states'] = state_hist['states'][state_hist_mask]

            #print "states", state_hist['states']
            

            if runtime.time() > PCA_HIST_LENGTH:
                projection = pca.project(state)
                print "proj", len(projection)
                projection = projection[:3]

                proj_hist['projs'] = np.append(proj_hist['projs'], [projection], axis = 0)
                proj_hist['times'] = np.append(proj_hist['times'], [runtime.time()], axis = 0)

                proj_hist_mask = np.where(proj_hist['times'] > max(proj_hist['times']) - PROJ_HIST_LENGTH)
                proj_hist['times'] = proj_hist['times'][proj_hist_mask]
                proj_hist['projs'] = proj_hist['projs'][proj_hist_mask]


            spike_hist_mask = np.where(spikes['times'] > max(spikes['times']) - SPIKE_HIST_LENGTH)
            spikes['times'] = spikes['times'][spike_hist_mask]
            spikes['senders'] = spikes['senders'][spike_hist_mask]

            d.on_running(spikes['times'], spikes['senders'], proj_hist['projs'])
        #print spikes
        print t, runtime.time()
        runtime.tick()
        t += 1 




if __name__ == "__main__":
    main()
