#include "ros_sensor_adapter.h"

void
ros_thread(RosSensorAdapter ros_adapter)
{
    ros_adapter.runROS();
}

int
main(int argc, char** argv)
{

    RosSensorAdapter ros_adapter;
    ros_adapter.init(argc, argv);

    boost::thread t = boost::thread(ros_thread, ros_adapter);

    ros_adapter.runMUSIC();
    t.join();

    ros_adapter.finalize();

}

void
RosSensorAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ROS sensor adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    sensor_update_rate = DEFAULT_SENSOR_UPDATE_RATE;

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
    subscriber = n.subscribe(ros_topic, 1000, &RosSensorAdapter::laserscanCallback, this);
}

void
RosSensorAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("message_type", &msg_type);
    setup->config("sensor_update_rate", &sensor_update_rate);
    setup->config("music_timestep", &timestep);

    MUSIC::ContOutputPort* port_out = setup->publishContOutput ("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       // which process am I?
    int nProcesses = comm.Get_size (); // how many processes are there?
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
      		 rank * datasize,
      		 datasize);
    port_out->map (&dmap, 1);
    
    runtime = new MUSIC::Runtime (setup, timestep);
}

void
RosSensorAdapter::runROS()
{
    std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;
    ros::Rate rate(sensor_update_rate); 
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void 
RosSensorAdapter::runMUSIC()
{
    ros::Rate rate(1/timestep);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();
        rate.sleep();
    }
}

void
RosSensorAdapter::laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    //std::cout << "Sensor: " ;
    //    std::cout  << data[i] << " " ;
    //std::cout << std::endl;
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        // scale data between -1 and 1
        // TODO: catch exception if ranges.size not width of port
        data[i] = (msg->ranges.at(i) / msg->range_max ) * 2 - 1;
    }
    //std::cout << msg->ranges.at(0) << " " << data[0] << " " << datasize << " " << msg->ranges.size() << std::endl;
    
}

void RosSensorAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}



