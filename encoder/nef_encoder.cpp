#include "nef_encoder.h"

int
main(int argc, char** argv)
{

    NefEncoder nef_encoder;
    nef_encoder.init(argc, argv);
    nef_encoder.runMUSIC();
    nef_encoder.finalize();

}

void
NefEncoder::init(int argc, char** argv)
{
    std::cout << "initializing nef encoder" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    initMUSIC(argc, argv);
}

void
NefEncoder::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);

    port_out = setup->publishEventOutput("out");
    port_in = setup->publishContInput("in");

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
        size_sensor_data = port_in->width();
        size_spike_data = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    for (unsigned int i = 0; i < size_sensor_data; ++i)
    {
        sensor_data.push_back(0.);
    }

    for (int n = 0; n < size_spike_data; ++n){
        IAFNeuron neuron;
        neuron.setResolution(timestep);
        neurons.push_back(neuron);
        neuron.encode(sensor_data);
    }

         
    // Declare where in memory to put sensor_data
    MUSIC::ArrayData dmap(&sensor_data[0],
      		 MPI::DOUBLE,
      		 rank * size_sensor_data,
      		 size_sensor_data);
    port_in->map (&dmap, 1);
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index_out(0, size_spike_data);
    port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);

    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
NefEncoder::runMUSIC()
{
    std::cout << "running nef encoder" << std::endl;
    struct timeval tval0;
    struct timeval tval1;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        gettimeofday(&tval0, NULL);
        runtime->tick();

        for (unsigned int n = 0; n < neurons.size(); ++n){
            neurons[n].encode(sensor_data);
            if (neurons[n].propagate()){
                //std::cout << "NEF: SPIKE!!" << std::endl;
                port_out->insertEvent(runtime->time(), MUSIC::GlobalIndex(n));
            }
        }


        gettimeofday(&tval1, NULL);
        unsigned int dt = tval1.tv_usec - tval0.tv_usec;
        if (tval1.tv_sec > tval0.tv_sec){
            dt += 1000000;
        }

        //std::cout << "dt " << dt << std::endl;
        if (dt < timestep * 1000000) // in us
            usleep(timestep * 1000000 - dt); // for the rest of the tick
    }
}

void NefEncoder::finalize(){
    runtime->finalize();
    delete runtime;
}




