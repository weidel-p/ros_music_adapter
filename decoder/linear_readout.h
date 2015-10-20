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

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_TAU = 0.03;
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
        int size_command_data;
        int size_spike_data;
        double* command_data;
        double* activity_traces;

        string weights_filename;
        Json::Value json_readout_weights; 
        double** readout_weights;
        MUSIC::EventInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        double tau, propagator;

        void initMUSIC(int argc, char** argv);
        void readWeightsFile();
        void operator() (double t, MUSIC::GlobalIndex id );
};


