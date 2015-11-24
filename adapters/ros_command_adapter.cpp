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
    msg_type = DEFAULT_MESSAGE_TYPE;

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
    switch (msg_type)
    {   
        case Float64MultiArray:
        {
            publisher = n.advertise<std_msgs::Float64MultiArray>(ros_topic, 1);
            break;
        }
        case Twist: 
        {
            publisher = n.advertise<geometry_msgs::Twist>(ros_topic, 1);
            break;
        }
    }
}

void
RosCommandAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("command_rate", &command_rate);
    setup->config("music_timestep", &timestep);
    
    setup->config("message_mapping_filename", &mapping_filename);
    readMappingFile();


    
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
    data = new double[datasize+1]; //+1 for the leading zero needed for unspecified fiels in the message 
    data[0] = 0.;
         
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (&data[1],
      		 MPI::DOUBLE,
      		 0,
      		 datasize);
    port_in->map (&dmap, 0., 1, false);
 
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
RosCommandAdapter::readMappingFile()
{
    Json::Reader json_reader;

    std::ifstream mapping_file;
    mapping_file.open(mapping_filename.c_str(), std::ios::in);
    string json_mapping_= "";
    string line;

    while (std::getline(mapping_file, line))
    {
        json_mapping_ += line;
    }
    mapping_file.close();
    
    if ( !json_reader.parse(json_mapping_, json_mapping))
    {
        // report to the user the failure and their locations in the document.
        std::cout   << "ERROR: ROS Command Adapter: Failed to parse file \"" << mapping_filename << "\"\n" 
                    << json_mapping_ << " It has to be in JSON format.\n"
                    << json_reader.getFormattedErrorMessages();
        return;
    }
    else
    {
        std::cout << json_mapping << std::endl;
        std::string _msg_type;
        _msg_type = json_mapping["message_type"].asString();
        if (_msg_type.compare("Float64MultiArray") == 0)
        {
            msg_type = Float64MultiArray;
            // no mapping needed
        }
        else if (_msg_type.compare("Twist") == 0)
        {
            msg_type = Twist;
            
            msg_map = new int[6];
            int index = -1;

            index = json_mapping["mapping"]["linear.x"].asInt();
            msg_map[0] = index + 1;

            index = -1;
            index = json_mapping["mapping"]["linear.y"].asInt();
            msg_map[1] = index + 1;

            index = -1;
            index = json_mapping["mapping"]["linear.z"].asInt();
            msg_map[2] = index + 1;

            index = -1;
            index = json_mapping["mapping"]["angular.x"].asInt();
            msg_map[3] = index + 1;

            index = -1;
            index = json_mapping["mapping"]["angular.y"].asInt();
            msg_map[4] = index + 1;

            index = -1;
            index = json_mapping["mapping"]["angular.z"].asInt();
            msg_map[5] = index + 1;
            std::cout << msg_map << std::endl;
        }
        else
        {
            std::cout << "ERROR: msg type unknown" << std::endl;
            finalize();
        }

    }

}

void
RosCommandAdapter::runROS()
{
    std::cout << "running command adapter with update rate of " << command_rate << std::endl;
    Rate rate(command_rate);
    ros::Time stop_time = ros::Time::now() + ros::Duration(stoptime);

    for (ros::Time t = ros::Time::now(); t < stop_time; t = ros::Time::now())
    {

        switch (msg_type)
        {   
            case Float64MultiArray:
            {
                std_msgs::Float64MultiArray msg;
                for (int i = 1; i < datasize+1; ++i)
                {
                    msg.data.push_back(data[i]);
                }
                publisher.publish(msg);
                break;
            }

            case Twist: 
            {
                geometry_msgs::Twist msg;
                
                msg.linear.x = data[msg_map[0]];
                msg.linear.y = data[msg_map[1]];
                msg.linear.z = data[msg_map[2]];

                msg.angular.x = data[msg_map[3]];
                msg.angular.y = data[msg_map[4]];
                msg.angular.z = data[msg_map[5]];
            
                publisher.publish(msg);
                break;
            }

        }
#if DEBUG_OUTPUT
        std::cout << "ROS Command Adapter: ";
        for (int i = 1; i < datasize + 1; ++i)
        {
        std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif


        ros::spinOnce();
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
        rate.sleep();
        runtime->tick();
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



