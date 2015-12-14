#include "latency_encoder.h"

int
main(int argc, char** argv)
{

    LatencyEncoder latency_encoder;
    latency_encoder.init(argc, argv);
    latency_encoder.runMUSIC();
    latency_encoder.finalize();

}

void
LatencyEncoder::init(int argc, char** argv)
{
    std::cout << "initializing latency encoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    initMUSIC(argc, argv);
}

void
LatencyEncoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);

    port_in = setup->publishContInput("in");
    port_out = setup->publishEventOutput("out");

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
        size_data = port_in->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    data = new double[size_data];
    for (int i = 0; i < size_data; ++i)
    {
        data[i] = 0.;
    }
         
    // Declare where in memory to put sensor_data
    MUSIC::ArrayData dmap(data,
      		 MPI::DOUBLE,
      		 rank * size_data,
      		 size_data);
    port_in->map (&dmap, timestep, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_out(0, size_data);
    port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
LatencyEncoder::runMUSIC()
{

    std::cout << "running latency encoder" << std::endl;
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    int ticks_skipped = 0;
    unsigned int num_spikes = 0;

    double t = runtime->time();
    while(t < stoptime)
    {
        runtime->tick();
        t = runtime->time();

        for (int n = 0; n < size_data; ++n)
        {
            num_spikes++;
            if (data[n] > 0.5){
                port_out->insertEvent(t, MUSIC::GlobalIndex(n));
#if DEBUG_OUTPUT
                std::cout << "Latency Encoder: neuron " << n << " spikes at "  << t << " for input " << data[n] << std::endl;
#endif
            }
        }
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "latency encoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped << " num spikes " << num_spikes << std::endl;
}

void
LatencyEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




