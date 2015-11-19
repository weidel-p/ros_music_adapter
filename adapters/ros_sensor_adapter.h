#include <iostream>
#include <map>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include <music.hh>
#include <mpi.h>

#include "boost/thread.hpp"
#include "sys/time.h"

#include "rtclock.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SENSOR_UPDATE_RATE = 30;

enum msg_types {Laserscan}; 

class RosSensorAdapter
{
    public:
        void init(int argc, char** argv);
	bool ratesMatch (double precision);
        void runMUSIC();
        void runROS();
	void runROSMUSIC();
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

        msg_types msg_type;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

        void laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg);

};
