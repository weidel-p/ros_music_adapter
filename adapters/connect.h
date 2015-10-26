#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"
#include "jsoncpp/json/json.h"
#include <iostream>
#include <fstream>

#include "../rate.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const string DEFAULT_WEIGHTS_FILENAME = "connection_weights.dat";

class ConnectAdapter{
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

        string weights_filename;
        Json::Value json_weights; 
        double** weights;

        MUSIC::ContInputPort* port_in;
        MUSIC::ContOutputPort* port_out;

        void initMUSIC(int argc, char** argv);
        void readWeightsFile();
};


