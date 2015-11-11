import sys
import os
import numpy as np
import json
import time

ITERATIONS = 2

data_filename = sys.argv[1] 

if os.path.exists(data_filename):
    os.remove(data_filename)

### OPTIMAL LATENCY CASE ###
music_config = \
        "stoptime=10.\n\
        [sensor]\n\
          binary=../ros_sensor_adapter\n\
          args=\n\
          np=1\n\
          music_timestep=0.001\n\
          ros_topic=/jubot/laserscan\n\
          message_type=Laserscan\n\
          sensor_update_rate=30\n\
        [diverse]\n\
          binary=../connect_adapter\n\
          args=\n\
          np=1\n\
          music_timestep=0.001\n\
          weights_filename=non_existing.dat\n\
        [encoder]\n\
          binary=../latency_encoder\n\
          args=\n\
          np=1\n\
          music_timestep=0.001\n\
        [decoder]\n\
          binary=../latency_decoder\n\
          args=\n\
          np=1\n\
          music_timestep=0.001\n\
        [command]\n\
          binary=../ros_command_adapter\n\
          args=\n\
          np=1\n\
          music_timestep=0.001\n\
          ros_topic=/jubot/cmd_vel\n\
          message_type=Twist\n\
          linear.x=0\n\
          command_rate=20\n\
        sensor.out->diverse.in[640]\n\
        diverse.out->encoder.in[1]\n\
        encoder.out->decoder.in[1]\n\
        decoder.out->command.in[1]"


music_config_file = open("config.music", 'w+')
music_config_file.writelines(music_config)
music_config_file.close()

data = {"iteration": [], "latency": [], "type": []}

for i in range(ITERATIONS):
    
    os.system("roslaunch jubot latency_measurement.launch &")
    time.sleep(1.)
    os.system("mpirun \-np 5 music config.music &")
    time.sleep(15.)
    os.system("kill $(pgrep ros)")
    time.sleep(2.)

    os.system("cp ~/.ros/command_times.dat .")

    command_data_file = open("command_times.dat", 'r')
    command_data = json.load(command_data_file)
    command_data_file.close()

    times = np.array(command_data['times']) - command_data['time_input_started']
    cmds = np.array(command_data['cmds'])
    
    data["latency"].append(times[np.min(np.where(cmds > 0))])
    data["iteration"].append(i)
    data["type"].append("optimized")

    print "LATENCY: ", data["latency"][-1]


if os.path.exists("config.music"):
    os.remove("config.music")

### STANDARD CASE ###
music_config = \
        "stoptime=10.\n\
        [sensor]\n\
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
          weights_filename=non_existing.dat\n\
        [encoder]\n\
          binary=../latency_encoder\n\
          args=\n\
          np=1\n\
          music_timestep=0.03333\n\
        [decoder]\n\
          binary=../latency_decoder\n\
          args=\n\
          np=1\n\
          music_timestep=0.05\n\
        [command]\n\
          binary=../ros_command_adapter\n\
          args=\n\
          np=1\n\
          music_timestep=0.05\n\
          ros_topic=/jubot/cmd_vel\n\
          message_type=Twist\n\
          linear.x=0\n\
          command_rate=20\n\
        sensor.out->diverse.in[640]\n\
        diverse.out->encoder.in[1]\n\
        encoder.out->decoder.in[1]\n\
        decoder.out->command.in[1]"


music_config_file = open("config.music", 'w+')
music_config_file.writelines(music_config)
music_config_file.close()

for i in range(ITERATIONS):
    
    os.system("roslaunch jubot latency_measurement.launch &")
    time.sleep(1.)
    os.system("mpirun \-np 5 music config.music &")
    time.sleep(15.)
    os.system("kill $(pgrep ros)")
    time.sleep(2.)

    os.system("cp ~/.ros/command_times.dat .")

    command_data_file = open("command_times.dat", 'r')
    command_data = json.load(command_data_file)
    command_data_file.close()

    times = np.array(command_data['times']) - command_data['time_input_started']
    cmds = np.array(command_data['cmds'])
    
    data["latency"].append(times[np.min(np.where(cmds > 0))])
    data["iteration"].append(i)
    data["type"].append("standard")

    print "LATENCY: ", data["latency"][-1]


if os.path.exists("config.music"):
    os.remove("config.music")


data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()

    

