import sys
import os
import numpy as np
import datetime
import json
import time

ITERATIONS = 2 
MIN_NUM_NEURONS = 50

sim_time = 10 # in sec

data_filename = sys.argv[1]

if os.path.exists(data_filename):
    os.remove(data_filename)


data ={"num_neurons": [], "time": [], "type": [], "iteration": []}

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
                  binary=./pyNEST_measurement.py\n\
                  args=-s 0.05 -t " + str(sim_time) + " -n " + str(num_neurons) + "\n\
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


def create_music_config_neuron(num_neurons, sim_time):
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
                [neuron]\n\
                  binary=nrniv\n\
                  np=2\n\
                  args=-nobanner -music -python \"NEURON_measurement.py\"\n\
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
                encoder.out->neuron.in[" + str(num_neurons) +"]\n\
                neuron.out->decoder.in[" + str(num_neurons) +"]\n\
                decoder.out->command.in[2]"

    if os.path.exists("config.music"):
        os.remove("config.music")

    music_config_file = open("config.music", 'w+')
    music_config_file.writelines(music_config)
    music_config_file.close()

    if os.path.exists("NEURON_measurement_params.dat"):
        os.remove("NEURON_measurement_params.dat")
    
    with open("NEURON_measurement_params.dat", "w") as f:
        json.dump({"t": 10., "n": num_neurons, "ts": 0.05, "s": 2}, f)

def start_ros():
    os.system("roslaunch jubot empty.launch &")
    time.sleep(5.)

def kill_ros():
    os.system("kill $(pgrep ros)")


#for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
#    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"
#
#    if num_neurons == 0:
#        num_neurons = 1
#
#    for it in range(ITERATIONS):
#
#        if os.path.exists("runtime.dat"):
#            os.remove("runtime.dat")
#
#        create_music_config_no_simulator(num_neurons, sim_time)
#
#        start_ros()
#
#        os.system("mpirun \-np 5 music config.music ")
#
#        kill_ros()
#        
#        with open("runtime.dat", 'r') as f:
#            run_time = float(json.load(f))
#
#        rtf = sim_time / run_time 
#
#        print
#        print
#        print rtf
#        print
#        print
#
#        insert_datapoint (num_neurons, rtf, "without neural simulator", it) 
# 
#for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
#    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"
#
#    if num_neurons == 0:
#        num_neurons = 1
#
#    for it in range(ITERATIONS):
#
#        if os.path.exists("runtime.dat"):
#            os.remove("runtime.dat")
#
#        create_music_config_nest(num_neurons, sim_time)
#        
#        start_ros()
#
#        os.system("mpirun \-np 12 music config.music ")
#
#        kill_ros()
#        
#        with open("runtime.dat", 'r') as f:
#            run_time = float(json.load(f))
#
#        rtf = sim_time / run_time 
#
#        print
#        print
#        print rtf
#        print
#        print
#
#        insert_datapoint (num_neurons, rtf, "with NEST", it) 
    

lower_limit = 0 
upper_limit = 2
end_loop = False

while True:

    mean_rtf = 0

    for it in range(ITERATIONS):
        print "\n\n\n\n\ RUNNING", upper_limit, "NEURONS \n\n\n\n"

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_neuron(upper_limit, sim_time)
        
        start_ros()

        os.system("mpirun \-np 7 music config.music ")

        kill_ros()
        
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print lower_limit, upper_limit, rtf
        print
        print

        insert_datapoint (upper_limit, rtf, "with NEURON", it) 

        mean_rtf += rtf / ITERATIONS

    if end_loop:
        break

    if mean_rtf > 0.95:
        tmp = upper_limit
        upper_limit += (upper_limit - lower_limit) * 2
        lower_limit = tmp 

    if mean_rtf < 0.95:
        upper_limit -= (upper_limit - lower_limit) / 2
        
    if lower_limit == upper_limit - 1:
        end_loop = True    

#for num_neurons in np.arange(MIN_NUM_NEURONS, MAX_NUM_NEURONS, STEP_SIZE):
#    print "\n\n\n\n\ RUNNING", num_neurons, "NEURONS \n\n\n\n"
#
#    if num_neurons == 0:
#        num_neurons = 1
#
#    for it in range(ITERATIONS):
#
#        if os.path.exists("runtime.dat"):
#            os.remove("runtime.dat")
#
#        create_music_config_neuron(num_neurons, sim_time)
#        
#        start_ros()
#
#        os.system("mpirun \-np 7 music config.music ")
#
#        kill_ros()
#        
#        with open("runtime.dat", 'r') as f:
#            run_time = float(json.load(f))
#
#        rtf = sim_time / run_time 
#
#        print
#        print
#        print rtf
#        print
#        print
#
#        insert_datapoint (num_neurons, rtf, "with NEURON", it) 
    

data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

