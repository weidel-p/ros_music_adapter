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
    grid_positions_filename = DEFAULT_GRID_POSITIONS_FILENAME;
    initMUSIC(argc, argv);
}

void
DiscretizeAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("grid_positions_filename", &grid_positions_filename);

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

    readGridPositionFile();

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


    MPI::COMM_WORLD.Barrier();
    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
DiscretizeAdapter::readGridPositionFile()
{
    Json::Reader json_reader;

    std::ifstream grid_positions_file;
    grid_positions_file.open(grid_positions_filename.c_str(), std::ios::in);
    string json_grid_positions_ = "";
    string line;

    while (std::getline(grid_positions_file, line))
    {
        json_grid_positions_+= line;
    }
    grid_positions_file.close();
    
    if ( !json_reader.parse(json_grid_positions_, json_grid_positions))
    {
      // report to the user the failure and their locations in the document.
      std::cout   << "WARNING: ros discretize adapter: Failed to parse file \"" << grid_positions_filename << "\"\n" 
		  << json_grid_positions_ << " It has to be in JSON format.\n "
		  << json_reader.getFormattedErrorMessages();
        
        return;
    }
    else
    {

        for (int i = 0; i < size_data_out; ++i)
        {
            double* pos_ = new double[size_data_in];
            double* sigmas_ = new double[size_data_in];

            for (int j = 0; j < size_data_in; ++j)
            {
                pos_[j] = json_grid_positions[i][j].asDouble();
               
                //put sigmas on the diagonal
                sigmas_[j] = json_grid_positions[i][size_data_in + j].asDouble(); 
            }
            grid_positions.insert(std::pair<int, double*>(i, pos_));
            sigmas.insert(std::pair<int, double*>(i, sigmas_));
        }


    }

}




void 
DiscretizeAdapter::runMUSIC()
{
    std::cout << "running discretize adapter" << std::endl;
    
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;



    for (int t = 0; runtime->time() < stoptime; t++)
    {
        for (int i = 0; i < size_data_out; ++i){
            double tmp_ = 0; 

            // calculate distance to this place cell 
            for (int j = 0; j < size_data_in; ++j){
                tmp_ += std::pow((data_in[j] - grid_positions[i][j]) / sigmas[i][j], 2);
            }
           
            // calculate activation in respect to gaussion kernel
            data_out[i] = -1. + 2. * std::exp(-tmp_/2.);
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




