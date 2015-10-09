#include <iostream>
#include <map>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include <music.hh>
#include <mpi.h>

#include "boost/thread.hpp"
#include "sys/time.h"

#include "rate.h"

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SENSOR_UPDATE_RATE = 30;

class RosSensorAdapter
{
    public:
        void init(int argc, char** argv);
        void runMUSIC();
        void runROS();
        void finalize();

    private:
        std::string ros_topic;
        ros::Subscriber subscriber;

        MPI::Intracomm comm;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;
        double sensor_update_rate;
        double timestep;

        double* data;

        std::string msg_type;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

        void laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg);

};
