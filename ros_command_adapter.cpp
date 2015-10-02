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
    
    MUSIC::ContInputPort* wavedata = setup->publishContInput ("commanddata");
    
    comm = setup->communicator ();
    int nProcesses = comm.Get_size (); // how many processes are there?
    int rank = comm.Get_rank ();       // which process am I?
    int width = 0;
    if (wavedata->hasWidth ())
      width = wavedata->width ();
    else
      comm.Abort (1);
    
    // For clarity, assume that width is a multiple of n_processes
    nLocalVars = width / nProcesses;
    data = new double[nLocalVars]; 
    databuf = new double[nLocalVars+1]; //+1 for the leading zero needed for unspecified fiels in the message
    databuf[0] = 0.;
         
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (data,
      		 MPI::DOUBLE,
      		 rank * nLocalVars,
      		 nLocalVars);
    wavedata->map (&dmap);
    
    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("message_type", &msg_type);

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
 
    runtime = new MUSIC::Runtime (setup, DEFAULT_TIMESTEP);

}

void
RosCommandAdapter::runROS()
{

    ros::Rate rate(30); 

    for (int t = 0; runtime->time() < stoptime; t++)
    {

//        std::cout << msg_type.compare("Twist") << std::endl;
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

        ros::spinOnce();
        rate.sleep();
       
    }

}

void 
RosCommandAdapter::runMUSIC()
{
    ros::Rate rate(1/DEFAULT_TIMESTEP);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();

        memcpy( &databuf[1], &data[0], nLocalVars * sizeof( double ) );

        for (int i = 0; i < nLocalVars + 1; ++i)
            std::cout << databuf[i] << ' ';
        std::cout << std::endl;

        rate.sleep();
    }

}

void RosCommandAdapter::finalize(){

    runtime->finalize();
    delete runtime;
}



