#include "zmq_in_adapter.h"

#include "rtclock.h"

static void*
zmq_thread(void* arg)
{
  ZmqInAdapter* zmq_adapter = static_cast<ZmqInAdapter*>(arg);
  zmq_adapter->runZMQ();
}


int
main(int argc, char** argv)
{

    ZmqInAdapter zmq_adapter;
    zmq_adapter.init(argc, argv);

    MPI::COMM_WORLD.Barrier();

    pthread_t t;
    pthread_create (&t, NULL, zmq_thread, &zmq_adapter);

    zmq_adapter.runMUSIC();
    pthread_join(t, NULL);

    zmq_adapter.finalize();

}


void
ZmqInAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ZMQ in adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    rtf = DEFAULT_RTF;
    zmq_addr = DEFAULT_ZMQ_ADDR;
    zmq_topic = DEFAULT_ZMQ_TOPIC;
    msg_type = DEFAULT_MESSAGE_TYPE;

    pthread_mutex_init(&data_mutex, NULL);
    // MUSIC before ZMQ to read the config first!
    initMUSIC(argc, argv);
}


void
ZmqInAdapter::initZMQ(int argc, char** argv)
{
}

void
ZmqInAdapter::initMUSIC(int argc, char** argv)
{
    setup = new MUSIC::Setup (argc, argv);

    setup->config("music_timestep", &timestep);
    setup->config("stoptime", &stoptime);
    setup->config("rtf", &rtf);
    setup->config("zmq_addr", &zmq_addr);
    setup->config("zmq_topic", &zmq_topic);

    std::string _msg_type;
    setup->config("message_type", &_msg_type);

    if (_msg_type.compare("ALEGrayScaleImage") == 0){
      msg_type = ALEGrayScaleImage;
    }
    else if (_msg_type.compare("FloatArray") == 0){
      msg_type = FloatArray;
    }
    else if (_msg_type.compare("GymObservation") == 0){
      msg_type = GymObservation;
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
ZmqInAdapter::runMUSIC()
{
    std::cout << "running zmq in adapter with update rate of " << 1./timestep << std::endl;
    RTClock clock(timestep / rtf);
  
    runtime = new MUSIC::Runtime (setup, timestep);
    
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        clock.sleepNext(); 

        runtime->tick();
    }

    std::cout << "sensor: total simtime: " << clock.time () << " s" << std::endl;
}

void 
ZmqInAdapter::runZMQ()
{ 
    zmq::context_t context(1);

    //  Connect our subscriber socket
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect(zmq_addr);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

 
    //  Get updates, expect random Ctrl-C death
    for (int t = 0; runtime->time() < stoptime; t++)
    {
        //std::cout << "RECV" << std::endl;
        Json::Value json_msg = s_recvAsJson (subscriber);


        if (msg_type == ALEGrayScaleImage){
            for (int i = 0; i < datasize; ++i)
            {
              // outgoing messages should be between -1 and 1. Grayscale pixels have to be treated accordingly
              data[i] = (json_msg[i].asDouble() / 127.) - 1;
            } 
        }

        if (msg_type == FloatArray){
            for (int i = 0; i < datasize; ++i)
            {
              //TODO check for input range and rescale to [-1, 1]
              data[i] = json_msg[i].asDouble() / 2.;
            } 
        }

        if (msg_type == GymObservation){

            struct timeval now_;
            gettimeofday(&now_, NULL);
            double ts_now = now_.tv_sec + now_.tv_usec/1000000.;

            int i = 0;
            Json::Value::iterator it = json_msg.begin();
            while (it != json_msg.end()){
                Json::Value v = (*it);
                data[i] = 2 * (v["value"].asFloat() - v["min"].asFloat()) / 
                          ((v["max"].asFloat() - v["min"].asFloat())) - 1;

                double t_diff = ts_now - v["ts"].asDouble();
                if (t_diff > 0.01){
                    std::cout << "WARNING: ZMQ_in_adapter " << zmq_addr << " might be out of sync" << std::endl;
                }

#if DEBUG_OUTPUT
                std::cout << v  << std::endl;
#endif
                ++it;
                ++i;
            }

        }
        
#if DEBUG_OUTPUT
        for (int i = 0; i < datasize; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
#endif

    }
}


void ZmqInAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}



