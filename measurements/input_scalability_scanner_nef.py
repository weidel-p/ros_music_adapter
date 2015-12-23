import sys
import os
import numpy as np
import datetime
import json
import time

ITERATIONS = 1 
MIN_NUM_NEURONS = 0 
MAX_NUM_NEURONS = 26001
STEP_SIZE = 2000

sim_time = 10 # in sec

data_filename = sys.argv[1]

if os.path.exists(data_filename):
    os.remove(data_filename)


data = {"num_neurons": [], "time": [], "type": [], "iteration": []}

def insert_datapoint(n, t, ty, i):
    data["num_neurons"].append(n)
    data['type'].append(ty)
    data['iteration'].append(i)
    data["time"].append(t)

def create_music_config_no_simulator(num_neurons, sim_time):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
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
                sensor.out->encoder.in[100]\n\
                encoder.out->decoder.in[" + str(num_neurons) +"]\n\
                decoder.out->command.in[2]"

    if os.path.exists("config.music"):
        os.remove("config.music")

    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()

def create_music_config_nest(num_neurons, sim_time):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                [nest]\n\
                  binary=./pyNEST_measurement.py\n\
                  args=-s 0.05 -t " + str(sim_time) + " -n " + str(num_neurons) + "\n\
                  np=1\n\
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
                sensor.out->encoder.in[100]\n\
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
    time.sleep(3.)

def kill_ros():
    os.system("kill $(pgrep ros)")


for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"

    if num_neurons == 0:
        num_neurons = 1

    for it in range(ITERATIONS):

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_no_simulator(num_neurons, sim_time)

        start_ros()

        os.system("mpirun \-np 4 music config.music ")

        kill_ros()

        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        insert_datapoint (num_neurons, rtf, "without neural simulator", it) 

        print
        print
        print num_neurons, rtf 
        print
        print

for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"

    if num_neurons == 0:
        num_neurons = 1

    for it in range(ITERATIONS):

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_nest(num_neurons, sim_time)

        start_ros()

        os.system("mpirun \-np 5 music config.music ")

        kill_ros()

        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        insert_datapoint (num_neurons, rtf, "with NEST", it) 

        print
        print
        print num_neurons, rtf 
        print
        print



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

