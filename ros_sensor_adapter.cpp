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

    MUSIC::ContOutputPort* wavedata = setup->publishContOutput ("sensordata");

    comm = setup->communicator ();
    int nProcesses = comm.Get_size (); // how many processes are there?
    int rank = comm.Get_rank ();       // which process am I?
    int width = 0;
    if (wavedata->hasWidth ())
      width = wavedata->width ();
    else
      comm.Abort (1);
    
    // For clarity, assume that width is a multiple of n_processes
    datasize = width / nProcesses;
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
    wavedata->map (&dmap);
    
 
    runtime = new MUSIC::Runtime (setup, DEFAULT_TIMESTEP);

}

void
RosSensorAdapter::runROS()
{
    ros::Rate rate(30); 
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void 
RosSensorAdapter::runMUSIC()
{
    ros::Rate rate(1/DEFAULT_TIMESTEP);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();
        rate.sleep();
    }

}

void
RosSensorAdapter::laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

    //memcpy(&data, &msg->ranges, datasize * sizeof(float));
    for (unsigned int i = 0; i < msg->ranges.size(); ++i){
        data[i] = msg->ranges.at(i);
    }
    //std::cout << msg->ranges.at(0) << " " << data[0] << " " << datasize << " " << msg->ranges.size() << std::endl;
    
}

void RosSensorAdapter::finalize(){

    
    runtime->finalize();
    delete runtime;
}



