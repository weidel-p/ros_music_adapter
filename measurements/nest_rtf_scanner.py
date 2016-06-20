#!/usr/bin/python

import os
import numpy as np
import time
import json
import sys


ITERATIONS = 2
sim_time = 1 # in sec

data_filename = sys.argv[1]
data = {"num_neurons": [], "time": [], "type": [], "iteration": []}

def insert_datapoint(n, t, ty, i):
    data["num_neurons"].append(n)
    data['type'].append(ty)
    data['iteration'].append(i)
    data["time"].append(t)


def start_ros():
    os.system("roslaunch jubot empty.launch &")
    time.sleep(5.)

def kill_ros():
    os.system("kill $(pgrep ros)")



def create_music_config_nest(num_neurons, num_neurons_brunel, sim_time):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=20\n\
                [connect]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                [encoder]\n\
                  binary=../rate_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  rate_min=1\n\
                  rate_max=2\n\
                [nest]\n\
                  binary=./pyNEST_measurement_with_brunel.py\n\
                  args=-s 0.05 -t " + str(sim_time) + " -n " + str(num_neurons) +  " -m " + str(num_neurons_brunel) + "\n\
                  np=7\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  music_acceptable_latency=0.05\n\
                  tau=0.03\n\
                [command]\n\
                  binary=../ros_command_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  ros_topic=/jubot/cmd_vel\n\
                  message_mapping_filename=twist_mapping.dat\n\
                  command_rate=20\n\
                sensor.out->connect.in[100]\n\
                connect.out->encoder.in[" + str(num_neurons) +"]\n\
                encoder.out->nest.in[" + str(num_neurons) +"]\n\
                nest.out->decoder.in[" + str(num_neurons) +"]\n\
                decoder.out->command.in[2]"

    if os.path.exists("config.music"):
        os.remove("config.music")

    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()

 
for n in np.arange(100, 1100, 300):

    mean_rtf = 0

    for it in range(ITERATIONS):
        print "\n\n\n\n\ RUNNING", n, "NEURONS \n\n\n\n"

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_nest(1000, n, sim_time)
        
        start_ros()

        os.system("mpirun \-np 12 music config.music ")

        kill_ros()
        
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print n*5, rtf
        print
        print

        insert_datapoint (n*5, rtf, "with ROS-MUSIC Toolchain", it) 

        mean_rtf += rtf / ITERATIONS


for n in np.arange(100, 2100, 300):

    for it in range(ITERATIONS):
        os.system("mpirun -np 7 python brunel_delta_nest.py " + str(n))
            
        with open("nest_alone_rtf.dat", 'r') as f:
            run_time = float(json.load(f))
            os.remove("nest_alone_rtf.dat")


        rtf = min(1., sim_time / run_time)

        print
        print
        print n*5, rtf
        print
        print

        # n * 5 as the brunel network uses that many neurons
        insert_datapoint (n*5, rtf, "NEST alone", it) 

 
data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()



