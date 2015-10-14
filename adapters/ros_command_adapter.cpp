#include "ros_command_adapter.h"


void
ros_thread(RosCommandAdapter ros_adapter)
{
    ros_adapter.runROS();
}

int
main(int argc, char** argv)
{

    RosCommandAdapter ros_adapter;
    ros_adapter.init(argc, argv);

    boost::thread t = boost::thread(ros_thread, ros_adapter);

    ros_adapter.runMUSIC();
    t.join();

    ros_adapter.finalize();

}

void
RosCommandAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ROS command adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    command_rate = DEFAULT_COMMAND_RATE;

    // MUSIC before ROS to read the config first!
    initMUSIC(argc, argv);
    initROS(argc, argv);
}


void
RosCommandAdapter::initROS(int argc, char** argv)
{
    ros::init(argc, argv, "ros_command_node");
    ros::start();

    ros::NodeHandle n;
    if (msg_type.compare("Twist") == 0)
    {
        publisher = n.advertise<geometry_msgs::Twist>(ros_topic, 1);
    }
    else
    {
        std::cout << "ERROR: msg type unknown" << std::endl;
        finalize();
    }
}

void
RosCommandAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("message_type", &msg_type);
    setup->config("command_rate", &command_rate);
    setup->config("music_timestep", &timestep);
    
    MUSIC::ContInputPort* port_in = setup->publishContInput ("in"); //TODO: read portname from file
    
    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    int width = 0;
    if (port_in->hasWidth ())
    {
        width = port_in->width ();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort (1);
    }
    
    datasize = width;
    data = new double[datasize]; 
    databuf = new double[datasize+1]; //+1 for the leading zero needed for unspecified fiels in the message
    databuf[0] = 0.;
         
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (data,
      		 MPI::DOUBLE,
      		 rank * datasize,
      		 datasize);
    port_in->map (&dmap, 1);

    if (msg_type.compare("Twist") == 0)
    {
        int index = -1;
        setup->config("linear.x", &index);
        msg_map.insert(std::make_pair("linear.x", index));

        index = -1;
        setup->config("linear.y", &index);
        msg_map.insert(std::make_pair("linear.y", index));

        index = -1;
        setup->config("linear.z", &index);
        msg_map.insert(std::make_pair("linear.z", index));

        index = -1;
        setup->config("angular.x", &index);
        msg_map.insert(std::make_pair("angular.x", index));

        index = -1;
        setup->config("angular.y", &index);
        msg_map.insert(std::make_pair("angular.y", index));

        index = -1;
        setup->config("angular.z", &index);
        msg_map.insert(std::make_pair("angular.z", index));
        
    } 
 
    runtime = new MUSIC::Runtime (setup, timestep);
}

void
RosCommandAdapter::runROS()
{
    std::cout << "running command adapter with update rate of " << command_rate << std::endl;
    Rate rate(command_rate);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        ros::spinOnce();

        if (msg_type.compare("Twist") == 0)
        {
            geometry_msgs::Twist command;
            
            command.linear.x = databuf[msg_map.find("linear.x")->second + 1];
            command.linear.y = databuf[msg_map.find("linear.y")->second + 1];
            command.linear.z = databuf[msg_map.find("linear.z")->second + 1];

            command.angular.x = databuf[msg_map.find("angular.x")->second + 1];
            command.angular.y = databuf[msg_map.find("angular.y")->second + 1];
            command.angular.z = databuf[msg_map.find("angular.z")->second + 1];
        
            publisher.publish(command);
        }
        rate.sleep();     
    }

}

void 
RosCommandAdapter::runMUSIC()
{

    Rate rate(1./timestep);
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();
        memcpy( &databuf[1], &data[0], datasize * sizeof( double ) );
        rate.sleep();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "command: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;


}

void RosCommandAdapter::finalize(){

    runtime->finalize();
    delete runtime;
}



