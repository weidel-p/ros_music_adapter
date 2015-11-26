#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;

class LatencyEncoder{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        int size_data;

        double* data;
        MUSIC::EventOutputPort* port_out;
        MUSIC::ContInputPort* port_in;

        void initMUSIC(int argc, char** argv);

};


