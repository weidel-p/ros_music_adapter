#include "zmq_out_adapter.h"


int
main(int argc, char** argv)
{

    ZmqOutAdapter zmq_adapter;
    zmq_adapter.init(argc, argv);

    MPI::COMM_WORLD.Barrier();

    zmq_adapter.runMUSIC();

    zmq_adapter.finalize();

}

void
ZmqOutAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ZMQ out adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    rtf = DEFAULT_RTF;
    zmq_addr = DEFAULT_ZMQ_ADDR;
    zmq_topic = DEFAULT_ZMQ_TOPIC;
    msg_type = DEFAULT_MESSAGE_TYPE;


    pthread_mutex_init(&data_mutex, NULL);

    // MUSIC before ZMQ to read the config first!
    initMUSIC(argc, argv);
    initZMQ(argc, argv);
}


void
ZmqOutAdapter::initZMQ(int argc, char** argv)
{
}

void
ZmqOutAdapter::initMUSIC(int argc, char** argv)
{
    setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("rtf", &rtf);
    setup->config("zmq_addr", &zmq_addr);
    setup->config("zmq_topic", &zmq_topic);

    std::string _msg_type;
    setup->config("message_type", &_msg_type);

    if (_msg_type.compare("FloatArray") == 0){
      msg_type = FloatArray;
    }
    else if (_msg_type.compare("GymCommand") == 0){
      msg_type = GymCommand;
    }
    
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
    data = new double[datasize]; //+1 for the leading zero needed for unspecified fiels in the message 
    for (unsigned int i = 0; i < datasize; ++i)
    {
        data[i] = 0.;
    }
    // Declare where in memory to put data
    MUSIC::ArrayData dmap (data,
      		 MPI::DOUBLE,
      		 0,
      		 datasize);
    port_in->map (&dmap, 0., 1, false);
}

void
ZmqOutAdapter::sendZMQ (zmq::socket_t &pub)
{
    Json::Value json_data(Json::arrayValue);

    if (msg_type == FloatArray){
        
        pthread_mutex_lock (&data_mutex);
        for (int i = 0; i < datasize; ++i){
           json_data.append(Json::Value(data[i]));
        }
	    pthread_mutex_unlock (&data_mutex);
        

    }
    else if (msg_type == GymCommand){

        pthread_mutex_lock (&data_mutex);
        for (unsigned int i = 0; i < datasize; ++i){
            Json::Value val;
            val["value"] = data[i];
            json_data.append(val);
        }
	    pthread_mutex_unlock (&data_mutex);
    }
    std::cout << writer.write(json_data) << std::endl;
    s_send (pub, writer.write(json_data));
}

void 
ZmqOutAdapter::runMUSIC()
{
    std::cout << "running zmq out adapter with update rate of " << timestep << std::endl;
    RTClock clock(timestep / rtf);

    zmq::context_t context(1);

    //  Connect our subscriber socket
    zmq::socket_t pub (context, ZMQ_PUB);
    pub.bind(zmq_addr.c_str());
 
    runtime = new MUSIC::Runtime (setup, timestep);
    
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        clock.sleepNext();

        sendZMQ(pub);
        
        runtime->tick();
    }

    std::cout << "out: total simtime: " << clock.time () << " s" <<  std::endl;
}

void ZmqOutAdapter::finalize()
{

    runtime->finalize();
    delete runtime;
}

