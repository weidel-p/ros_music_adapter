#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "sys/time.h"
#include "jsoncpp/json/json.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;


class LatencyDecoder: MUSIC::EventHandlerGlobalIndex{
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
        int num_spikes;

        MUSIC::EventInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        void initMUSIC(int argc, char** argv);
        void operator() (double t, MUSIC::GlobalIndex id );
};


