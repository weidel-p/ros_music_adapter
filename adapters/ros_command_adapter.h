#include <iostream>
#include <map>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <music.hh>
#include <mpi.h>

#include "boost/thread.hpp"
#include "sys/time.h"

#include "rate.h"

#define DEBUG_OUTPUT false 

enum msg_types {Float64MultiArray, Twist};

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_COMMAND_RATE = 10;
const msg_types DEFAULT_MESSAGE_TYPE = Float64MultiArray;

class RosCommandAdapter
{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void runROS();
        void finalize();

    private:
        std::string ros_topic;
        ros::Publisher publisher;

        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;

        double* data;

        double timestep;
        double command_rate;

        msg_types msg_type;
        int* msg_map;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

};
