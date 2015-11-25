import sys
import os
import numpy as np
import datetime
import json

ITERATIONS = 2 
MIN_FIRING_RATE = 0
MAX_FIRING_RATE = 101
STEP_SIZE = 10

sim_time = 10 # in sec
sim_time_build = 0.001 # in sec

num_neurons = 50000

data_filename = sys.argv[1] 

if os.path.exists(data_filename):
    os.remove(data_filename)

data = {"time": [], "type": [], "iteration": [], "firing_rate": []}

for firing_rate in np.arange(MIN_FIRING_RATE, MAX_FIRING_RATE, STEP_SIZE):
    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS WITH A FIRING RATE OF", firing_rate, "\n\n\n\n"

    if firing_rate == 0:
        firing_rate = 1

    music_base_config = \
                "[sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [diverse]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                [encoder]\n\
                  binary=../rate_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.03333\n\
                  rate_min=" + str(firing_rate) + "\n\
                  rate_max=" + str(firing_rate) + "\n\
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
                  message_mapping_filename=float_mapping.dat\n\
                  command_rate=20\n\
                sensor.out->diverse.in[640]\n\
                diverse.out->encoder.in[" + str(num_neurons) + "]\n\
                encoder.out->decoder.in[" + str(num_neurons) + "]\n\
                decoder.out->command.in[2]"

    music_config_build = "stoptime=" + str(sim_time_build) + "\n"\
                         + music_base_config

    music_config_build_file = open("config_build.music", 'w+')
    music_config_build_file.writelines(music_config_build)
    music_config_build_file.close()

    music_config_run = "stoptime=" + str(sim_time) + "\n"\
                       + music_base_config

    music_config_run_file = open("config_run.music", 'w+')
    music_config_run_file.writelines(music_config_run)
    music_config_run_file.close()

    for it in range(ITERATIONS):

        start = datetime.datetime.now()
        os.system("mpirun \-np 5 music config_build.music ")
        end = datetime.datetime.now()

        dt_build = end - start
        build_time = dt_build.seconds + dt_build.microseconds / 1000000.

        data["firing_rate"].append(firing_rate)
        data['type'].append("build-time")
        data['iteration'].append(it)
        data["time"].append(build_time)

        start = datetime.datetime.now()
        os.system("mpirun \-np 5 music config_run.music ")
        end = datetime.datetime.now()
        
        dt_run = end - start
        run_time = dt_run.seconds + dt_run.microseconds / 1000000.

        data["firing_rate"].append(firing_rate)
        data['type'].append("total-time")
        data['iteration'].append(it)
        data["time"].append(run_time)

        rtf = sim_time / (run_time - build_time)

        data["firing_rate"].append(firing_rate)
        data['type'].append("real-time factor")
        data['iteration'].append(it)
        data["time"].append(rtf)
    
    if os.path.exists("config_run.music"):
        os.remove("config_run.music")
    if os.path.exists("config_build.music"):
        os.remove("config_build.music")



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

