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
    weights_filename = DEFAULT_WEIGHTS_FILENAME;
    tau = DEFAULT_TAU;

    // init MUSIC to read config
    initMUSIC(argc, argv); 
    readWeightsFile();
}

void
LinearReadoutDecoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("weights_filename", &weights_filename);

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
LinearReadoutDecoder::readWeightsFile()
{
    Json::Reader json_reader;

    std::ifstream weights_file;
    weights_file.open(weights_filename.c_str(), std::ios::in);
    string json_weights = "";
    string line;

    while (std::getline(weights_file, line))
    {
        json_weights += line;
    }
    weights_file.close();
    
    if ( !json_reader.parse(json_weights, readout_weights))
    {
        // report to the user the failure and their locations in the document.
        std::cout   << "ERROR: linear readout: Failed to parse file \"" << weights_filename << "\"\n" 
                    << json_weights << " It has to be in JSON format.\n"
                    << json_reader.getFormattedErrorMessages();

        comm.Abort(1);
        return;
    }

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


    // REMOVE ME
//    
//    readout_weights[0][0] = 0.12;
//    readout_weights[0][1] = 0.12;
//    readout_weights[1][0] = -0.5;
//    readout_weights[1][1] = 0.5;
//
    //

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();

        for (int i = 0; i < size_spike_data; ++i)
        {
            activity_traces[i] *= propagator;
        } 
       
        for (int i = 0; i < size_command_data; ++i)
        {
            command_data[i] = 0.;
            for (int j = 0; j < size_spike_data; ++j)
            {
                command_data[i] += activity_traces[j] * readout_weights[i][j].asDouble();
            }
        }
        //std::cout << "decoder: acti " << activity_traces[0] << " " << activity_traces[1] << std::endl;
        //std::cout << "decoder: command" << command_data[0] << " " << command_data[1] << std::endl;
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




