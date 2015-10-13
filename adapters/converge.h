#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"

#include "../rate.h"

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_TAU = 0.03;

class ConvergeAdapter{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        int size_data_in;
        int size_data_out;
        double* data_in;
        double* data_out;

        MUSIC::ContInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        void initMUSIC(int argc, char** argv);
};


