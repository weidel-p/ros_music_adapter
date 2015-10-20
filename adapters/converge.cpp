#include "converge.h"

int
main(int argc, char** argv)
{

    ConvergeAdapter conv_adapter;
    conv_adapter.init(argc, argv);
    conv_adapter.runMUSIC();
    conv_adapter.finalize();

}

void
ConvergeAdapter::init(int argc, char** argv)
{
    std::cout << "initializing converge adapter" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    initMUSIC(argc, argv);
}

void
ConvergeAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);

    port_in = setup->publishContInput("in");
    port_out = setup->publishContOutput("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    // get dimensions of data
    if (port_in->hasWidth() && port_out->hasWidth())
    {
        size_data_in = port_in->width();
        size_data_out = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    data_in = new double[size_data_in];
    for (int i = 0; i < size_data_in; ++i)
    {
        data_in[i] = 0.;
    }

    data_out = new double[size_data_out];
    for (int i = 0; i < size_data_out; ++i)
    {
        data_out[i] = 0.;
    }

         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap_in(data_in,
      		 MPI::DOUBLE,
      		 rank * size_data_in,
      		 size_data_in);
    port_in->map (&dmap_in, 1);
    
    MUSIC::ArrayData dmap_out(data_out,
      		 MPI::DOUBLE,
      		 rank * size_data_out,
      		 size_data_out);
    port_out ->map (&dmap_out, 1);

    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
ConvergeAdapter::runMUSIC()
{
    std::cout << "running converge adapter" << std::endl;
    
    Rate rate(1./timestep);
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    double size_factor = double(size_data_in) / size_data_out;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();
        for (int i = 0; i < size_data_out; ++i)
        {
            data_out[i] = 0;
            for (int j = i * size_factor; j < (i+1) * size_factor; ++j)
            {
                data_out[i] += data_in[j] * (1. / size_factor);
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
    std::cout << "converge adapter: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

void
ConvergeAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}




