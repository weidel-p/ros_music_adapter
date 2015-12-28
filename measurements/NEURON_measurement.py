from neuron import h
from neuronmusic import *
from mpi4py import MPI
import json
import numpy as np

with open("NEURON_measurement_params.dat", "r+") as f:
    params = json.load(f)

to_ms = lambda t: t * 1000.

comm = MPI.COMM_WORLD
comm_self = MPI.COMM_NULL

h.load_file('stdrun.hoc')

class Cell(object):
  def __init__(self):
    self.topology()
    self.subsets()
    self.geometry()
    self.biophys()
    self.synapses()

  def topology(self):
    self.soma = h.Section(cell = self)
    self.dend = h.Section(cell = self)
    self.dend.connect(self.soma)
    self.nseg = 5

  def subsets(self):
    self.all = h.SectionList()
    self.all.wholetree(sec=self.soma)

  def geometry(self):
    self.soma.L = 10
    self.soma.diam = 10
    self.dend.L = 500
    self.dend.diam = 1

  def biophys(self):
    for sec in self.all:
      sec.Ra = 100
      sec.cm = 1
    self.soma.insert('hh')

  def synapses(self):
    self.syn = h.ExpSyn(.5, sec = self.dend)
    self.syn.e = 0
    self.syn.tau = 1

  def connectToTarget(self, syn):
    nc = h.NetCon(self.soma(.5)._ref_v, syn, sec = self.soma)
    nc.threshold = -10
    return nc


out = publishEventOutput ('out')
inp = publishEventInput ('in')

cells = np.empty(params["n"]) 
ncs = np.empty(params["n"])

cells = []
ncs = []

_id = 0
pc = h.ParallelContext()
for i in range(params["n"]):
    if not i % params["s"] == pc.id():
        continue
    _ind = i + 1
    cells.append(Cell())
    pc.set_gid2node(_ind, pc.id())
    pc.cell(_ind, cells[_id].connectToTarget(None))
    out.gid2index(_ind, i)

    ncs.append(inp.index2target(_id, cells[_id].syn))
    ncs[_id].weight[0] = 5.
    ncs[_id].delay = 0
    _id += 1


#
#v_vec = h.Vector()             # Membrane potential vector
#t_vec = h.Vector()             # Time stamp vector
#v_vec.record(cells[0].soma(0.5)._ref_v)
#t_vec.record(h._ref_t)
#



pc.set_maxstep(to_ms(params["ts"]))
comm.Barrier()
h.stdinit()
pc.psolve(to_ms(params["t"]))

#from matplotlib import pyplot
#pyplot.figure(figsize=(8,4)) # Default figsize is (8,6)
#pyplot.plot(t_vec, v_vec)
#pyplot.xlabel('time (ms)')
#pyplot.ylabel('mV')
#pyplot.show()


