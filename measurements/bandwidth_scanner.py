import sys
import os
import numpy as np
import datetime
import json
import time

ITERATIONS = 5 
MIN_FIRING_RATE = 0
MAX_FIRING_RATE = 101
STEP_SIZE = 5

sim_time = 10 # in sec

num_neurons = 10000

data_filename = sys.argv[1] 

if os.path.exists(data_filename):
    os.remove(data_filename)

data = {"time": [], "type": [], "iteration": [], "firing_rate": []}

def insert_datapoint(f, t, ty, i):
    data["firing_rate"].append(f)
    data['type'].append(ty)
    data['iteration'].append(i)
    data["time"].append(t)

def create_music_config_no_simulator(num_neurons, sim_time, firing_rate):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
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
                  rate_min=" + str(firing_rate) + "\n\
                  rate_max=" + str(firing_rate) + "\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
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
                encoder.out->decoder.in[" + str(num_neurons) +"]\n\
                decoder.out->command.in[2]"

    if os.path.exists("config.music"):
        os.remove("config.music")

    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()

def create_music_config_nest(num_neurons, sim_time, firing_rate):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
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
                  rate_min=" + str(firing_rate) + "\n\
                  rate_max=" + str(firing_rate) + "\n\
                [nest]\n\
                  binary=./pyNEST_measurement.py\n\
                  args=-s 0.05 -t " + str(sim_time) + " -n " + str(num_neurons) + "\n\
                  np=7\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
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

def start_ros():
    os.system("roslaunch jubot empty.launch &")
    time.sleep(5.)

def kill_ros():
    os.system("kill $(pgrep ros)")



for firing_rate in np.arange(MIN_FIRING_RATE, MAX_FIRING_RATE, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS WITH A FIRING RATE OF", firing_rate, "\n\n\n\n"

    if firing_rate == 0:
        firing_rate = 1

    for it in range(ITERATIONS):

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_no_simulator(num_neurons, sim_time, firing_rate)

        start_ros()

        os.system("mpirun \-np 5 music config.music ")

        kill_ros()
 
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print rtf
        print
        print

        insert_datapoint (firing_rate, rtf, "without neural simulator", it) 

    

for firing_rate in np.arange(MIN_FIRING_RATE, MAX_FIRING_RATE, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS WITH A FIRING RATE OF", firing_rate, "\n\n\n\n"

    if firing_rate == 0:
        firing_rate = 1

    for it in range(ITERATIONS):

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_nest(num_neurons, sim_time, firing_rate)

        start_ros()

        os.system("mpirun \-np 12 music config.music ")

        kill_ros()
 
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print rtf
        print
        print

        insert_datapoint (firing_rate, rtf, "with NEST", it) 


data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

