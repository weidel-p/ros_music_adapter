import sys
import os
import numpy as np
import datetime
import json

ITERATIONS = 2 
MIN_NUM_NEURONS = 0
MAX_NUM_NEURONS = 5001
STEP_SIZE = 5000

sim_time = 10 # in sec
sim_time_build = 0.001 # in sec

data_filename = sys.argv[1]

if os.path.exists(data_filename):
    os.remove(data_filename)


data ={"num_neurons": [], "time": [], "type": [], "iteration": []}

def insert_datapoint(n, t, ty, i):
    data["num_neurons"].append(n)
    data['type'].append(ty)
    data['iteration'].append(i)
    data["time"].append(t)

def create_music_config(num_neurons, sim_time):
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
                  music_acceptable_latency=0.05\n\
                  tau=0.03\n\
                [command]\n\
                  binary=../ros_command_adapter\n\
                  args=\n\
                  np=1\n\
                  usic_timestep=0.05\n\
                  ros_topic=/jubot/cmd_vel\n\
                  message_mapping_filename=twist_mapping.dat\n\
                  command_rate=20\n\
                sensor.out->encoder.in[100]\n\
                encoder.out->decoder.in[" + str(num_neurons) +"]\n\
                decoder.out->command.in[2]"
    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()


for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"

    if num_neurons == 0:
        num_neurons = 1

    for it in range(ITERATIONS):

        create_music_config(num_neurons, sim_time_build)
        start = datetime.datetime.now()
        os.system("mpirun \-np 4 music config.music ")
        end = datetime.datetime.now()
        dt_build = end - start
        build_time = dt_build.seconds + dt_build.microseconds / 1000000.
        insert_datapoint (num_neurons, build_time, "build-time", it) 

        create_music_config(num_neurons, sim_time)
        start = datetime.datetime.now()
        os.system("mpirun \-np 4 music config.music ")
        end = datetime.datetime.now()
        dt_run = end - start
        run_time = dt_run.seconds + dt_run.microseconds / 1000000.
        insert_datapoint (num_neurons, run_time, "total-time", it) 

        rtf = sim_time / (run_time - build_time)
        insert_datapoint (num_neurons, rtf, "real-time factor", it) 

    
    if os.path.exists("config.music"):
        os.remove("config.music")



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

