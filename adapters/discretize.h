#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"
#include <json/json.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <numeric>

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SIGMA = 0.1;

class DiscretizeAdapter{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void finalize();

    private:
        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        double timestep;
        double sigma;
        double spacing;
        int num_place_cells_per_dim;
        double* tmp_place_cell_pos;

        int size_data_in;
        int size_data_out;
        double* data_in;
        double* data_out;

        MUSIC::ContInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        void initMUSIC(int argc, char** argv);
};


