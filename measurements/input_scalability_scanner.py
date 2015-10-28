import sys
import os
import numpy as np
import datetime
import json

ITERATIONS = 2 
MIN_NUM_NEURONS = 1
MAX_NUM_NEURONS = 5002
STEP_SIZE = 5000

run_time = 10 # in sec
run_time_build = 0.001 # in sec

data_filename = sys.argv[1]

if os.path.exists(data_filename):
    os.remove(data_filename)


data ={"num_neurons": [], "build_time": [], "run_time": [], "real-time_factor": []}

for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
    music_base_config = \
                "music_timestep=0.001\n\
                [sensor]\n\
                  binary=../ros_sensor_adapter\n\
                  args=\n\
                  np=1\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=30\n\
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
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
                sensor.out->encoder.in[640]\n\
                encoder.out->decoder.in[" + str(num_neurons) +"]\n\
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
        os.system("mpirun \-np 4 music config_build.music ")
        end = datetime.datetime.now()

        dt_build = end - start
        data["build_time"].append((dt_build.seconds * 1000000 + dt_build.microseconds) / 1000000.)

        start = datetime.datetime.now()
        os.system("mpirun \-np 4 music config_run.music ")
        end = datetime.datetime.now()
        
        dt_run = end - start
        data["run_time"].append((dt_run.seconds * 1000000 + dt_run.microseconds) / 1000000.)

        data["num_neurons"].append(num_neurons)

        rtf = run_time / (data["run_time"][-1] - data["build_time"][-1])
        data["real-time_factor"].append(rtf)
    
    if os.path.exists("config_run.music"):
        os.remove("config_run.music")
    if os.path.exists("config_build.music"):
        os.remove("config_build.music")



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

