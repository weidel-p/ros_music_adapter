#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "sys/time.h"
#include "jsoncpp/json/json.h"

#include "../rate.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_ACCEPTABLE_LATENCY = 1e-3;
const double DEFAULT_TAU = 0.03;
const double DEFAULT_NEURON_RESOLUTION = 1e-3;
const string DEFAULT_WEIGHTS_FILENAME = "readout_weights.dat";


class LinearReadoutDecoder : MUSIC::EventHandlerGlobalIndex{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        double acceptable_latency;
        int size_command_data;
        int size_spike_data;
        double* command_data;
        double* activity_traces;
        unsigned int num_spikes0, num_spikes1;

        string weights_filename;
        Json::Value json_readout_weights; 
        double** readout_weights;
        MUSIC::EventInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        double tau, propagator;
        std::map<double, std::vector<int> > incoming_spikes; 

        void initMUSIC(int argc, char** argv);
        void readWeightsFile();
        void operator() (double t, MUSIC::GlobalIndex id );
};


