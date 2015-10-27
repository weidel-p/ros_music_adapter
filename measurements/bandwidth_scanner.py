import sys
import os
import numpy as np
import datetime
import json

ITERATIONS = 2 
MIN_FIRING_RATE = 1
MAX_FIRING_RATE = 102
STEP_SIZE = 100

run_time = 10 # in sec
run_time_build = 0.1 # in sec

num_neurons = 100

data_filename = "bandwidth.dat"

if os.path.exists(data_filename):
    os.remove(data_filename)

data = {"build_time": [], "run_time": [], "real-time_factor": [], "firing_rate": []}

for firing_rate in np.arange(MIN_FIRING_RATE, MAX_FIRING_RATE, STEP_SIZE):
    music_base_config = \
                "music_timestep=0.001\n\
                [sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [diverse]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                [encoder]\n\
                  binary=../rate_encoder\n\
                  args=\n\
                  np=1\n\
                  rate_min=" + str(firing_rate) + "\n\
                  rate_max=" + str(firing_rate) + "\n\
                [decoder]\n\
                  binary=../linear_readout_decoder\n\
                  args=\n\
                  np=1\n\
                  tau=0.03\n\
                [command]\n\
                  binary=../ros_command_adapter\n\
                  args=\n\
                  np=1\n\
                  ros_topic=/jubot/cmd_vel\n\
                  message_type=Twist\n\
                  linear.x=0\n\
                  angular.z=1\n\
                  command_rate=20\n\
                sensor.out->diverse.in[640]\n\
                diverse.out->encoder.in[" + str(num_neurons) + "]\n\
                encoder.out->decoder.in[" + str(num_neurons) + "]\n\
                decoder.out->command.in[2]"

    music_config_build = "stoptime=" + str(run_time_build) + "\n"\
                         + music_base_config

    music_config_build_file = open("config_build.music", 'w+')
    music_config_build_file.writelines(music_config_build)
    music_config_build_file.close()

    music_config_run = "stoptime=" + str(run_time) + "\n"\
                       + music_base_config

    music_config_run_file = open("config_run.music", 'w+')
    music_config_run_file.writelines(music_config_run)
    music_config_run_file.close()


    for _ in range(ITERATIONS):
        start = datetime.datetime.now()
        os.system("mpirun \-np 5 music config_build.music ")
        end = datetime.datetime.now()

        dt_build = end - start
        data["build_time"].append((dt_build.seconds * 1000000 + dt_build.microseconds) / 1000000.)

        start = datetime.datetime.now()
        os.system("mpirun \-np 5 music config_run.music ")
        end = datetime.datetime.now()
        
        dt_run = end - start
        data["run_time"].append((dt_run.seconds * 1000000 + dt_run.microseconds) / 1000000.)

        data["firing_rate"].append(firing_rate)

        rtf = run_time / (data["run_time"][-1] - data["build_time"][-1])
        data["real-time_factor"].append(rtf)
    
    if os.path.exists("config_run.music"):
        os.remove("config_run.music")
    if os.path.exists("config_build.music"):
        os.remove("config_build.music")



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

