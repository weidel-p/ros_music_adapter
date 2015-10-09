#include "linear_readout.h"

int
main(int argc, char** argv)
{

    LinearReadoutDecoder lin_encoder;
    lin_encoder.init(argc, argv);
    lin_encoder.runMUSIC();
    lin_encoder.finalize();

}

void
LinearReadoutDecoder::init(int argc, char** argv)
{
    std::cout << "initializing linear readout decoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    tau = DEFAULT_TAU;
    initMUSIC(argc, argv);
}

void
LinearReadoutDecoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("tau", &tau);

    port_in = setup->publishEventInput("in");
    port_out = setup->publishContOutput("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       // which process am I?
    int nProcesses = comm.Get_size (); // how many processes are there?
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

    activity_traces = new double[size_spike_data];
    for (int i = 0; i < size_spike_data; ++i)
    {
        activity_traces[i] = 0.;
    }

         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap(command_data,
      		 MPI::DOUBLE,
      		 rank * size_command_data,
      		 size_command_data);
    port_out->map (&dmap, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_in(0, size_spike_data);
    port_in->map(&l_index_in, this, 0.002, 1); //TODO evaluate 0.002


    // initialize propagator for exponential decay
    propagator = std::exp(-(timestep)/tau);

    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
LinearReadoutDecoder::runMUSIC()
{
    std::cout << "running linear readout decoder" << std::endl;
    
    Rate rate(1./timestep);
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();

        for (int i = 0; i < size_spike_data; ++i)
        {
            activity_traces[i] *= propagator;
        } 
       
        for (int i = 0; i < size_command_data; ++i)
        {
            //TODO use weights
            for (int j = 0; j < size_spike_data; ++j)
            {
                command_data[i] += activity_traces[j] * (1. / size_spike_data);
            }
        }
        rate.sleep();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "decoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;

}

void LinearReadoutDecoder::operator () (double t, MUSIC::GlobalIndex id){
    // Decoder: update neural activity traces
    activity_traces[id] += 1;

}
void LinearReadoutDecoder::finalize(){
    runtime->finalize();
    delete runtime;
}




