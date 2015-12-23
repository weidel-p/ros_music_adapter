import sys
import os
import numpy as np
import datetime
import json
import time

ITERATIONS = 5 
MIN_NUM_NEURONS = 0
MAX_NUM_NEURONS = 200001
STEP_SIZE = 5000

MIN_TIMESTEP = 0.000
MAX_TIMESTEP = 0.051
TIMESTEP_STEP_SIZE = 0.005


sim_time = 10 # in sec

data_filename = sys.argv[1]

if os.path.exists(data_filename):
    os.remove(data_filename)

data = {"num_neurons": [], "time": [], "type": [], "iteration": [], "timestep": []}

def insert_datapoint(n, t, ty, i, ts):
    data["num_neurons"].append(n)
    data['type'].append(ty)
    data['iteration'].append(i)
    data["time"].append(t)
    data["timestep"].append(ts)

def create_music_config_no_simulator(num_neurons, sim_time, timestep):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [connect]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                [encoder]\n\
                  binary=../rate_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  rate_min=1\n\
                  rate_max=2\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  tau=0.03\n\
                [command]\n\
                  binary=../ros_command_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
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

def create_music_config_nest(num_neurons, sim_time, timestep):
    music_config = \
                "stoptime=" + str(sim_time) + "\n"\
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [connect]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                [encoder]\n\
                  binary=../rate_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  rate_min=1\n\
                  rate_max=2\n\
                [nest]\n\
                  binary=./pyNEST_measurement.py\n\
                  args=-s " + str(timestep) + " -t " + str(sim_time) + " -n " + str(num_neurons) + "\n\
                  np=7\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  tau=0.03\n\
                [command]\n\
                  binary=../ros_command_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
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


for timestep in np.arange(MIN_TIMESTEP, MAX_TIMESTEP, TIMESTEP_STEP_SIZE):
    last_rtf = 1
    for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):

        if timestep == 0:
            timestep = 0.001
        if num_neurons == 0:
            num_neurons = 1

        print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS WITH TIMESTEP", timestep, "\n\n\n\n"

        for it in range(ITERATIONS):
    
            if last_rtf < 0.2:
                # fill data up with zeros if computation takes too long
                insert_datapoint (num_neurons, 0, "no neural simulator", it, timestep) 
                continue

            create_music_config_no_simulator(num_neurons, sim_time, timestep)

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
    
            insert_datapoint (num_neurons, rtf, "no neural simulator", it, timestep) 

            last_rtf = min(rtf, last_rtf)
        

data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()



#for timestep in np.arange(MIN_TIMESTEP, MAX_TIMESTEP, TIMESTEP_STEP_SIZE):
#    last_rtf = 1
#    for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
#
#        if timestep == 0:
#            timestep = 0.001
#        if num_neurons == 0:
#            num_neurons = 1
#
#        print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS WITH TIMESTEP", timestep, "\n\n\n\n"
#
#        for it in range(ITERATIONS):
#    
#            if last_rtf < 0.2:
#                # fill data up with zeros if computation takes too long
#                insert_datapoint (num_neurons, 0, "with NEST", it, timestep) 
#                continue
#
#            create_music_config_nest(num_neurons, sim_time, timestep)
#            os.system("mpirun \-np 12 music config.music ")
#             
#            with open("runtime.dat", 'r') as f:
#                run_time = float(json.load(f))
#
#            rtf = sim_time / run_time 
#
#            print
#            print
#            print rtf
#            print
#            print
#    
#            insert_datapoint (num_neurons, rtf, "with NEST", it, timestep) 
#
#            last_rtf = min(rtf, last_rtf)
#        
#            if os.path.exists("config.music"):
#                os.remove("config.music")
#
#
#
#data_file = open(data_filename, "w+")
#json.dump(data, data_file)
#data_file.close()


    
        
    

