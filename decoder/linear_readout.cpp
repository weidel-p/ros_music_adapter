#include "linear_readout.h"

int
main(int argc, char** argv)
{

    LinearReadoutDecoder lin_decoder;
    lin_decoder.init(argc, argv);
    lin_decoder.runMUSIC();
    lin_decoder.finalize();

}

void
LinearReadoutDecoder::init(int argc, char** argv)
{
    std::cout << "initializing linear readout decoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    acceptable_latency = DEFAULT_ACCEPTABLE_LATENCY;
    weights_filename = DEFAULT_WEIGHTS_FILENAME;
    tau = DEFAULT_TAU;
    num_spikes0 = 0;
    num_spikes1 = 0;

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
    setup->config("music_acceptable_latency", &acceptable_latency);
    setup->config("tau", &tau);

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

    activity_traces = new double[size_spike_data];
    for (int i = 0; i < size_spike_data; ++i)
    {
        activity_traces[i] = 0.;
    }

    readout_weights = new double*[size_command_data];
    for (int i = 0; i < size_command_data; ++i)
    {
        readout_weights[i] = new double[size_spike_data];
    }
         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap(command_data,
      		 MPI::DOUBLE,
      		 rank * size_command_data,
      		 size_command_data);
    port_out->map (&dmap, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_in(0, size_spike_data);
    port_in->map(&l_index_in, this, acceptable_latency, 1); 


    // initialize propagator for exponential decay
    propagator = std::exp(-(DEFAULT_NEURON_RESOLUTION)/tau);

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
    
    if ( !json_reader.parse(json_weights, json_readout_weights))
    {
        // report to the user the failure and their locations in the document.
        std::cout   << "ERROR: linear readout: Failed to parse file \"" << weights_filename << "\"\n" 
                    << json_weights << " It has to be in JSON format.\n Using 1/N for each weight."
                    << json_reader.getFormattedErrorMessages();
        
        for (int i = 0; i < size_command_data; ++i)
        {
            for (int j = 0; j < size_spike_data; ++j)
            {
                readout_weights[i][j] = 1. / size_spike_data;
            }
        }

        return;
    }
    else
    {
        for (int i = 0; i < size_command_data; ++i)
        {
            for (int j = 0; j < size_spike_data; ++j)
            {
                readout_weights[i][j] = json_readout_weights[i][j].asDouble();
            }
        }

    }

}

void 
LinearReadoutDecoder::runMUSIC()
{
    std::cout << "running linear readout decoder" << std::endl;
    
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);

    std::map<double, std::vector<int> >::iterator it, it_now;
    double t = runtime->time();
    while(t < stoptime)
    {
        t = runtime->time();
        
        double next_t = t + timestep; //time at next timestep
        while (t <= next_t) // update the activity traces in higher resolution than the MUSIC timestep
        {
            it_now = incoming_spikes.lower_bound(t); // get all incoming spikes until t
            if (it_now != incoming_spikes.end()) 
            {
                for (it = incoming_spikes.begin(); it != it_now; ++it)
                {
                    std::vector<int> ids = it->second;
                    for (std::vector<int>::iterator i = ids.begin(); i != ids.end(); ++i)
                    {
                        activity_traces[*i] += 1 / tau;
                        ///std::cout << runtime->time() << " " << t << " " << *i << std::endl;
                        num_spikes1++;
                    }
                }
                incoming_spikes.erase(incoming_spikes.begin(), it_now);
            }

            for (int i = 0; i < size_spike_data; ++i)
            {
                activity_traces[i] *= propagator;
            } 
            t += DEFAULT_NEURON_RESOLUTION;
        }
       
        for (int i = 0; i < size_command_data; ++i)
        {
            command_data[i] = 0.;
            for (int j = 0; j < size_spike_data; ++j)
            {
                command_data[i] += activity_traces[j] * readout_weights[i][j];
            }
        }

#if DEBUG_OUTPUT
        std::cout << "Linear Readout: Activity Traces: ";
        for (int i = 0; i < size_spike_data; ++i)
        {
            std::cout << activity_traces[i] << " ";
        }
        std::cout << std::endl;


        std::cout << "Linear Readout: Command Data: ";
        for (int i = 0; i < size_command_data; ++i)
        {
            std::cout << command_data[i] << " ";
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
    std::cout << "decoder: total simtime: " << dt_s << " " << dt_us << " received spikes " << num_spikes0  << " filtered spikes " << num_spikes1 << std::endl;

}

void LinearReadoutDecoder::operator () (double t, MUSIC::GlobalIndex id){
    // Decoder: add incoming spikes to map
    num_spikes0++;

    // check if a spike with the same timestamp is already in the map
    std::map<double, std::vector<int> >::iterator it = incoming_spikes.find(t + acceptable_latency);
    if (it != incoming_spikes.end()) // if so
    {
        it->second.push_back(id); // add new spike to vector
    }
    else // if not
    {
        std::vector<int> ids; // create new vector and add to map
        ids.push_back(id);
        incoming_spikes.insert(std::make_pair(t + acceptable_latency, ids)); // insert spike with delay of one acceptable latency 
    }

}
void LinearReadoutDecoder::finalize(){
    runtime->finalize();
    delete runtime;
}




