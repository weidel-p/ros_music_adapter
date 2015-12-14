#include "latency_decoder.h"

int
main(int argc, char** argv)
{

    LatencyDecoder latency_decoder;
    latency_decoder.init(argc, argv);
    latency_decoder.runMUSIC();
    latency_decoder.finalize();

}

void
LatencyDecoder::init(int argc, char** argv)
{
    std::cout << "initializing latency decoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    num_spikes = 0;

    // init MUSIC to read config
    initMUSIC(argc, argv); 
}

void
LatencyDecoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);

    port_in = setup->publishEventInput("in");
    port_out = setup->publishContOutput("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    // get dimensions of sensory data and spike data
    if (port_in->hasWidth() && port_out->hasWidth())
    {
        size_spike_data = port_in->width();
        size_command_data = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    command_data = new double[size_command_data];
    for (int i = 0; i < size_command_data; ++i)
    {
        command_data[i] = 0.;
    }
         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap(command_data,
      		 MPI::DOUBLE,
      		 rank * size_command_data,
      		 size_command_data);
    port_out->map (&dmap, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_in(0, size_spike_data);
    port_in->map(&l_index_in, this, timestep, 1); 

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
LatencyDecoder::runMUSIC()
{
    std::cout << "running latency decoder" << std::endl;
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);

    double t = runtime->time();
    while(t < stoptime)
    {
        runtime->tick();
        t = runtime->time();
        

        for (int i = 0; i < size_command_data; ++i)
        {
            if (num_spikes == 0)
                command_data[i] = 0.;
            else
                command_data[i] = 1.;
        }

#if DEBUG_OUTPUT
        std::cout << "Latency Decoder: Command Data: ";
        for (int i = 0; i < size_command_data; ++i)
        {
            std::cout << command_data[i] << " ";
        }
        std::cout << std::endl;
#endif
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "decoder: total simtime: " << dt_s << " " << dt_us << std::endl;

}

void LatencyDecoder::operator () (double t, MUSIC::GlobalIndex id){
    // Decoder: add incoming spikes to map
    num_spikes++;
}
void LatencyDecoder::finalize(){
    runtime->finalize();
    delete runtime;
}




