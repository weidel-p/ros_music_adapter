#include <iostream>
#include <map>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <music.hh>
#include <mpi.h>

#include "boost/thread.hpp"
#include "sys/time.h"

#include "rate.h"

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_COMMAND_RATE = 10;
enum msg_types {Twist};

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
        double* databuf;

        double timestep;
        double command_rate;

        msg_types msg_type;
//        std::map<std::string, int> msg_map; 
        int* msg_map;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

};
