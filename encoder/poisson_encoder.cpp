#include "rate_encoder.h"

int
main(int argc, char** argv)
{

    RateEncoder rate_encoder;
    rate_encoder.init(argc, argv);
    rate_encoder.runMUSIC();
    rate_encoder.finalize();

}

void
RateEncoder::init(int argc, char** argv)
{
    std::cout << "initializing rate encoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    initMUSIC(argc, argv);
}

void
RateEncoder::initMUSIC(int argc, char** argv)
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
    
    rates = new double[size_data];
    rates_buf = new double[size_data];
    next_spike = new double[size_data];
    for (int i = 0; i < size_data; ++i)
    {
        rates[i] = 8.;
        next_spike[i] = 0.01; //negexp(denormalize(rates[i])); 
    }
    rates_buf = rates;
         
    // Declare where in memory to put sensor_data
    MUSIC::ArrayData dmap(rates,
      		 MPI::DOUBLE,
      		 rank * size_data,
      		 size_data);
    port_in->map (&dmap, timestep, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_out(0, size_data);
    port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);

    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
RateEncoder::runMUSIC()
{

    std::cout << "running rate encoder" << std::endl;
    Rate rate(1./timestep);
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    int ticks_skipped = 0;
    double t = runtime->time();

    while(t < stoptime)
    {
        t = runtime->time();
        runtime->tick();
        if (rates != rates_buf) 
        {
            for (int n = 0; n < size_data; ++n)
            {
                next_spike[n] += 1./(rates[n] + 1) / 100.; //negexp(denormalize(rates[n]));
            }
            rates_buf = rates;
        }

        for (int n = 0; n < size_data; ++n)
        {
            while(next_spike[n] < t + timestep)
            {
#if DEBUG_OUTPUT
                std::cout << "Poisson Encoder: neuron " << n << " spiked at " << runtime->time() << std::endl;
#endif
                port_out->insertEvent(runtime->time(), MUSIC::GlobalIndex(n));
                next_spike[n] += 1./(rates[n] + 1) / 100.; //negexp(denormalize(rates[n]));
            }
        }

        //std::cout << "rate encoder: " << next_spike[0] << " " << next_spike[1] << std::endl;
        //std::cout << "rate encoder rates: " << rates[0] << " " << rates[1] << std::endl;
        rate.sleep();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "rate encoder: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

double
RateEncoder::denormalize(double s)
{
  return 1. / ((s + 1) * 8.); 
}

double
RateEncoder::negexp (double m)
{
  return - m * log (drand48 ());
}

void
RateEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




