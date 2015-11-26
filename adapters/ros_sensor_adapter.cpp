#include "ros_sensor_adapter.h"

#include "rtclock.h"

static void*
ros_thread(void* arg)
{
    RosSensorAdapter* ros_adapter = static_cast<RosSensorAdapter*>(arg);
    ros_adapter->runROS();
}

int
main(int argc, char** argv)
{

    RosSensorAdapter ros_adapter;
    ros_adapter.init(argc, argv);

    // If sensor_update_rate and timestep match to a relative
    // precision of 0.1%, lump the ROS and MUSIC event loops
    // together.
    if (ros_adapter.ratesMatch (0.001))
    {
	    ros_adapter.runROSMUSIC();
    }
    else
    {
        pthread_t t;
	pthread_create (&t, NULL, ros_thread, &ros_adapter);

    	ros_adapter.runMUSIC();
    	pthread_join(t, NULL);
    }

    ros_adapter.finalize();

}


bool
RosSensorAdapter::ratesMatch (double precision)
{
    return std::abs (sensor_update_rate * timestep - 1.) < precision;
}


void
RosSensorAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ROS sensor adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    sensor_update_rate = DEFAULT_SENSOR_UPDATE_RATE;

    pthread_mutex_init(&data_mutex, NULL);

    // MUSIC before ROS to read the config first!
    initMUSIC(argc, argv);
    initROS(argc, argv);
}


void
RosSensorAdapter::initROS(int argc, char** argv)
{
    ros::init(argc, argv, "ros_sensor_node");
    ros::start();

    ros::NodeHandle n;
    switch (msg_type)
    {
        case Laserscan:
            subscriber = n.subscribe(ros_topic, 1000, &RosSensorAdapter::laserscanCallback, this);
            break;
    }
}

void
RosSensorAdapter::initMUSIC(int argc, char** argv)
{
    setup = new MUSIC::Setup (argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("sensor_update_rate", &sensor_update_rate);
    setup->config("music_timestep", &timestep);

    std::string _msg_type;
    setup->config("message_type", &_msg_type);

    if (_msg_type.compare("Laserscan") == 0){
        msg_type = Laserscan;
    }
    else
    {
        std::cout << "ERROR: msg type unknown" << std::endl;
        finalize();
    }


    MUSIC::ContOutputPort* port_out = setup->publishContOutput ("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }


    if (port_out->hasWidth ())
    {
        datasize = port_out->width ();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort (1);
    }
    
    data = new double[datasize]; 
    for (unsigned int i = 0; i < datasize; ++i)
    {
        data[i] = 0.;
    }
         
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (data,
      		 MPI::DOUBLE,
      		 0,
      		 datasize);
    port_out->map (&dmap, 1);
}

void
RosSensorAdapter::runROSMUSIC()
{
    RTClock clock(1. / sensor_update_rate);
    
    ros::spinOnce();
    runtime = new MUSIC::Runtime (setup, timestep);

    for (int t = 0; runtime->time() < stoptime; t++)
    {

#if DEBUG_OUTPUT
        std::cout << "ROS Sensor Adapter: ";
        for (int i = 0; i < datasize; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif

        clock.sleepNext(); 
        ros::spinOnce();
        runtime->tick();
    }

    std::cout << "sensor: total simtime: " << clock.time () << " s" << std::endl;
}

void
RosSensorAdapter::runROS()
{
    RTClock clock(1. / sensor_update_rate);
    ros::Time stop_time = ros::Time::now() + ros::Duration(stoptime);
    std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;

    ros::spinOnce();
    for (ros::Time t = ros::Time::now(); t < stop_time; t = ros::Time::now())
    {
        clock.sleepNext();
        ros::spinOnce();
   }
}

void 
RosSensorAdapter::runMUSIC()
{
    RTClock clock(timestep);

    runtime = new MUSIC::Runtime (setup, timestep);
    
    for (int t = 0; runtime->time() < stoptime; t++)
    {

#if DEBUG_OUTPUT
        std::cout << "ROS Sensor Adapter: ";
        for (int i = 0; i < datasize; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif

        clock.sleepNext(); 
	pthread_mutex_lock(&data_mutex);
        runtime->tick();
	pthread_mutex_unlock(&data_mutex);
    }

    std::cout << "sensor: total simtime: " << clock.time () << " s" << std::endl;
}

void
RosSensorAdapter::laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pthread_mutex_lock(&data_mutex);
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        // scale data between -1 and 1
        // TODO: catch exception if ranges.size not width of port
        data[i] = ((msg->ranges.at(i) - msg->range_min) / (msg->range_max - msg->range_min) ) * 2 - 1;
    }
    pthread_mutex_unlock(&data_mutex);    
}

void RosSensorAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}



