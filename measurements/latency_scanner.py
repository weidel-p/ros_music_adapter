import sys
import os
import numpy as np
import json
import time

ITERATIONS = 5
MIN_TIMESTEP = 0.0
MAX_TIMESTEP = 0.051
TIMESTEP_STEP_SIZE = 0.01

sim_time = 10 # in sec

data_filename = sys.argv[1] 

if os.path.exists(data_filename):
    os.remove(data_filename)

data = {"iteration": [], "latency": [], "timestep": [], "type": []}

def insert_datapoint(i, lat, ts, ty):
    data['iteration'].append(i)
    data['latency'].append(lat)
    data["timestep"].append(ts)
    data["type"].append(ty)

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
                  sensor_update_rate=20\n\
                [connect]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                [encoder]\n\
                  binary=../latency_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  rate_min=1\n\
                  rate_max=2\n\
                [decoder]\n\
                  binary=../latency_decoder\n\
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
                  sensor_update_rate=20\n\
                [connect]\n\
                  binary=../connect_adapter\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                [encoder]\n\
                  binary=../latency_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep="+ str(timestep) +"\n\
                  rate_min=1\n\
                  rate_max=2\n\
                [nest]\n\
                  binary=./pyNEST_measurement.py\n\
                  args=-s " + str(timestep) + " -t " + str(sim_time) + " -n " + str(num_neurons) + "\n\
                  np=1\n\
                [decoder]\n\
                  binary=../latency_decoder\n\
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

    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()



for timestep in np.arange(MIN_TIMESTEP, MAX_TIMESTEP, TIMESTEP_STEP_SIZE):
    if timestep == 0:
            timestep = 0.001

    create_music_config_no_simulator(1, sim_time, timestep)
    
    for it in range(ITERATIONS):
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
        latency = times[np.min(np.where(cmds > 0))]

        insert_datapoint(it, latency, timestep, "without neural simulator")

        print "LATENCY: ", data["latency"][-1]


    if os.path.exists("config.music"):
        os.remove("config.music")

for timestep in np.arange(MIN_TIMESTEP, MAX_TIMESTEP, TIMESTEP_STEP_SIZE):
    if timestep == 0:
            timestep = 0.001

    create_music_config_nest(1, sim_time, timestep)
    
    for it in range(ITERATIONS):
        os.system("roslaunch jubot latency_measurement.launch &")
        time.sleep(1.)
        os.system("mpirun \-np 6 music config.music &")
        time.sleep(15.)
        os.system("kill $(pgrep ros)")
        time.sleep(2.)

        os.system("cp ~/.ros/command_times.dat .")

        command_data_file = open("command_times.dat", 'r')
        command_data = json.load(command_data_file)
        command_data_file.close()

        times = np.array(command_data['times']) - command_data['time_input_started']
        cmds = np.array(command_data['cmds'])
        latency = times[np.min(np.where(cmds > 0))]

        insert_datapoint(it, latency, timestep, "with NEST")

        print "LATENCY: ", data["latency"][-1]


    if os.path.exists("config.music"):
        os.remove("config.music")



data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()










    

