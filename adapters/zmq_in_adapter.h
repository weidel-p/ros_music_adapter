#include <iostream>
#include <map>
#include <math.h>

#include <music.hh>
#include <mpi.h>

#include "sys/time.h"

#include <iostream>
#include "zhelpers.hpp"
#include "jsoncpp/json/json.h"

#define DEBUG_OUTPUT false 

enum msg_types {ALEGrayScaleImage, FloatArray, GymObservation};

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_RTF = 1.0;
const std::string DEFAULT_ZMQ_ADDR = "tcp://localhost:5555";
const std::string DEFAULT_ZMQ_TOPIC = "in";

const msg_types DEFAULT_MESSAGE_TYPE = ALEGrayScaleImage;

class ZmqInAdapter
{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void runZMQ();
        void finalize();

    private:
        std::string zmq_addr;
        std::string zmq_topic;
        //zmq::context_t context(1);
        //zmq::socket_t subscriber; 
        double rtf;

        MPI::Intracomm comm;
	    MUSIC::Setup* setup;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;
        double timestep;
        msg_types msg_type;

        pthread_mutex_t data_mutex;
        double* data;

        void initZMQ(int argc, char** argv);
        void initMUSIC(int argc, char** argv);
};
