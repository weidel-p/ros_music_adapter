import sys
import os
import numpy as np
import datetime
import json
import time

ITERATIONS = 5 

rtf_threshold = 0.95
num_processes_NEURON = 40

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
                  music_timestep=0.05\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=20\n\
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
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
                  music_timestep=0.05\n\
                  ros_topic=/jubot/laserscan\n\
                  message_type=Laserscan\n\
                  sensor_update_rate=20\n\
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
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
                [encoder]\n\
                  binary=../nef_encoder\n\
                  args=\n\
                  np=1\n\
                  music_timestep=0.05\n\
                [neuron]\n\
                  binary=nrniv\n\
                  np=" + str(num_processes_NEURON) + "\n\
                  args=-nobanner -music -python NEURON_measurement.py\n\
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
        json.dump({"t": 10., "n": num_neurons, "ts": 0.05, "s": num_processes_NEURON}, f)




def start_ros():
    os.system("roslaunch jubot empty.launch &")
    time.sleep(3.)

def kill_ros():
    os.system("kill $(pgrep ros)")


global lower_limit, upper_limit, known_upper_limit
lower_limit = 0 
upper_limit = num_processes_NEURON #minimum is one neuron per process in NEURON  
known_upper_limit = sys.maxint

def bin_search(rtf):
    global lower_limit, upper_limit, known_upper_limit
    accuracy = 100
    if rtf > rtf_threshold:
        tmp = upper_limit
        upper_limit += (upper_limit - lower_limit) * 2
        lower_limit = tmp 

        if upper_limit >= known_upper_limit:
            upper_limit = lower_limit + (known_upper_limit - lower_limit) / 2

    else:
        known_upper_limit = upper_limit
        upper_limit -= int(np.ceil( (upper_limit - lower_limit) / 2.))
        
    if lower_limit in range(upper_limit - accuracy, upper_limit + accuracy) and known_upper_limit < sys.maxint: # upper limit found and accuarcy reached
        return True

    return False



while True:

    mean_rtf = 0

    for it in range(ITERATIONS):
        print "\n\n\n\n\ RUNNING", upper_limit, "NEURONS \n\n\n\n"

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_no_simulator(upper_limit, sim_time)
        
        start_ros()

        os.system("mpirun \-np 4 music config.music ")

        kill_ros()
        
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print lower_limit, upper_limit, rtf
        print
        print

        insert_datapoint (upper_limit, rtf, "without neural simulator", it) 

        mean_rtf += rtf / ITERATIONS

    if bin_search(mean_rtf):
        break


 
lower_limit = 0 
upper_limit = num_processes_NEURON 
known_upper_limit = sys.maxint

while True:

    mean_rtf = 0

    for it in range(ITERATIONS):
        print "\n\n\n\n\ RUNNING", upper_limit, "NEURONS \n\n\n\n"

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_nest(upper_limit, sim_time)
        
        start_ros()

        os.system("mpirun \-np 5 music config.music ")

        kill_ros()
        
        with open("runtime.dat", 'r') as f:
            run_time = float(json.load(f))

        rtf = sim_time / run_time 

        print
        print
        print lower_limit, upper_limit, rtf
        print
        print

        insert_datapoint (upper_limit, rtf, "with NEST", it) 

        mean_rtf += rtf / ITERATIONS

    if bin_search(mean_rtf):
        break
       

lower_limit = 0 
upper_limit = num_processes_NEURON 
known_upper_limit = sys.maxint

while True:

    mean_rtf = 0

    for it in range(ITERATIONS):
        print "\n\n\n\n\ RUNNING", upper_limit, "NEURONS \n\n\n\n"

        if os.path.exists("runtime.dat"):
            os.remove("runtime.dat")

        create_music_config_neuron(upper_limit, sim_time)
        
        start_ros()

        os.system("mpirun \-np " + str(num_processes_NEURON + 4) + " music config.music ")

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

    if bin_search(mean_rtf):
        break


data_file = open(data_filename, "w+")
json.dump(data, data_file)
data_file.close()


    
        
    

