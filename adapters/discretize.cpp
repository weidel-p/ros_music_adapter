#include "discretize.h"

int
main(int argc, char** argv)
{

    DiscretizeAdapter discretize_adapter;
    discretize_adapter.init(argc, argv);
    discretize_adapter.runMUSIC();
    discretize_adapter.finalize();

}

void
DiscretizeAdapter::init(int argc, char** argv)
{
    std::cout << "initializing discretize adapter" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    sigma = DEFAULT_SIGMA;
    initMUSIC(argc, argv);
}

void
DiscretizeAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("sigma", &sigma);

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
             0,
             size_data_in);
    port_in->map (&dmap_in, 0., 1, false);
    
    MUSIC::ArrayData dmap_out(data_out,
             MPI::DOUBLE,
             0,
             size_data_out);
    port_out ->map (&dmap_out, 1);


    // distance between "place cells":
    // input has a range of 2 (between -1 and +1),
    // number of "place cells" per dimension is m^(1/n)
    // where n is the dimensionality and 
    // m is the total number of place cells
    num_place_cells_per_dim = int(std::pow(size_data_out, 1. / size_data_in));
    spacing = 2. / num_place_cells_per_dim;

    tmp_place_cell_pos = new double[size_data_in]; 
    

    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
DiscretizeAdapter::runMUSIC()
{
    std::cout << "running discretize adapter" << std::endl;
    
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    double* tmp_dist = new double[size_data_in]; 
    for (int i = 0; i < size_data_in; ++i)
    {
        tmp_dist[i] = 0.;
    }


    for (int t = 0; runtime->time() < stoptime; t++)
    {
        for (int i = 0; i < size_data_in; ++i)
        {
            tmp_place_cell_pos[i] = -1;
        }

        for (int i = 0; i < size_data_out; ++i){

            // calculate distance to this place cell 
            for (int j = 0; j < size_data_in; ++j){
                tmp_dist[j] = data_in[j] - tmp_place_cell_pos[j];
            }

            // calculate activation in respect to gaussion kernel
            data_out[i] = -1. + 2. * std::exp(
                   -std::inner_product(tmp_dist, &tmp_dist[size_data_in], tmp_dist, 0.0)
                   / std::pow(sigma, 2) );

            // update position of place cell
            tmp_place_cell_pos[0] += spacing;
            for (int j = 1; j < size_data_in; ++j){
                if ( i % (int)std::pow(num_place_cells_per_dim, j) == 0)
                {
                    tmp_place_cell_pos[j-1] = -1;
                    tmp_place_cell_pos[j] += spacing;
                }
            }
            
        }
   
       
#if DEBUG_OUTPUT
        std::cout << "Discretize Adapter: ";
        for (int i = 0; i < size_data_out; ++i)
        {
            std::cout << data_out[i] << " ";
        }
        std::cout << std::endl;
#endif
        runtime->tick();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "discretize adapter: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

void
DiscretizeAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}




