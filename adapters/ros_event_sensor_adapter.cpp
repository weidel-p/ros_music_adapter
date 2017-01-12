#include "ros_event_sensor_adapter.h"

#include "rtclock.h"

#include "dvs_msgs/Event.h"

#define BENCHMARK_ROS_CALLBACK 0
#define DEBUG_TICK 0
#define EVENT_TRANSFORMATION 0

#if BENCHMARK_ROS_CALLBACK
#include <time.h>
#endif

static void*
ros_thread(void* arg) {
	RosEventSensorAdapter* ros_adapter = static_cast<RosEventSensorAdapter*> (arg);
	ros_adapter->runROS();
}

int
main(int argc, char** argv) {

	RosEventSensorAdapter ros_adapter;
	ros_adapter.init(argc, argv);

	MPI::COMM_WORLD.Barrier();
	// If sensor_update_rate and timestep match to a relative
	// precision of 0.1%, lump the ROS and MUSIC event loops
	// together.
	if (ros_adapter.ratesMatch(0.001)) {
		ros_adapter.runROSMUSIC();
	} else {
		pthread_t t;
		pthread_create(&t, NULL, ros_thread, &ros_adapter);

		ros_adapter.runMUSIC();
		pthread_join(t, NULL);
	}

	ros_adapter.finalize();

}

bool
RosEventSensorAdapter::ratesMatch(double precision) {
	return std::abs(sensor_update_rate * timestep - 1.) < precision;
}

void
RosEventSensorAdapter::init(int argc, char** argv) {
	std::cout << "initializing ROS event sensor adapter" << std::endl;

	timestep = DEFAULT_TIMESTEP;
	sensor_update_rate = DEFAULT_SENSOR_UPDATE_RATE;
	ros_node_name = DEFAULT_ROS_NODE_NAME;
	rtf = DEFAULT_RTF;

	// MUSIC before ROS to read the config first!
	initMUSIC(argc, argv);
	initROS(argc, argv);
}

void
RosEventSensorAdapter::initROS(int argc, char** argv) {
	ros::init(argc, argv, ros_node_name);
	ros::start();

	ros::NodeHandle n;
	subscriber = n.subscribe(ros_topic, 1000, &RosEventSensorAdapter::eventArrayCallback, this);
}

void
RosEventSensorAdapter::initMUSIC(int argc, char** argv) {
	setup = new MUSIC::Setup(argc, argv);

	setup->config("ros_topic", &ros_topic);
	setup->config("stoptime", &stoptime);
	setup->config("sensor_update_rate", &sensor_update_rate);
	setup->config("ros_node_name", &ros_node_name);
	setup->config("rtf", &rtf);

	port_out = setup->publishEventOutput("out");

	comm = setup->communicator();
	int rank = comm.Get_rank();
	int nProcesses = comm.Get_size();
	if (nProcesses > 1) {
		std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
		comm.Abort(1);
	}

	// map linear index to event out port
	MUSIC::LinearIndex l_index_out(0, port_out->width());
	port_out->map(&l_index_out, MUSIC::Index::GLOBAL, 1);
}

void
RosEventSensorAdapter::runROSMUSIC() {
	std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;
	RTClock clock(1. / (sensor_update_rate * rtf));

	ros::spinOnce();
	runtime = new MUSIC::Runtime(setup, timestep);

	for (int t = 0; runtime->time() < stoptime; t++) {
		last_tick_time = runtime->time();
		clock.sleepNext();
		ros::spinOnce();
#if DEBUG_TICK
		std::cout << "event sensor adapter: tick " << runtime->time() << std::endl;
#endif
		runtime->tick();
	}

	std::cout << "sensor: total simtime: " << clock.time() << " s" << std::endl;
}

void
RosEventSensorAdapter::runROS() {
	RTClock clock(1. / (sensor_update_rate * rtf));

	// wait until first sensor update arrives
	while (ros::Time::now().toSec() == 0.) {
		clock.sleepNext();
	}

	ros::Time stop_time = ros::Time::now() + ros::Duration(stoptime / rtf);

	ros::spinOnce();
	for (ros::Time t = ros::Time::now(); t < stop_time; t = ros::Time::now()) {
		clock.sleepNext();
		ros::spinOnce();
	}
}

void
RosEventSensorAdapter::runMUSIC() {
	std::cout << "running event sensor adapter with update rate of " << sensor_update_rate << std::endl;
	RTClock clock(timestep / rtf);

	runtime = new MUSIC::Runtime(setup, timestep);

	for (int t = 0; runtime->time() < stoptime; t++) {
		last_tick_time = runtime->time();
		clock.sleepNext();
#if DEBUG_TICK
		std::cout << "event sensor adapter: tick " << runtime->time() << std::endl;
#endif
		runtime->tick();
	}

	std::cout << "sensor: total simtime: " << clock.time() << " s" << std::endl;
}

void
RosEventSensorAdapter::eventArrayCallback(const dvs_msgs::EventArray msg) {
#if BENCHMARK_ROS_CALLBACK
	struct timespec start, finish;
	double elapsed;
	clock_gettime(CLOCK_MONOTONIC, &start);
#endif
	// msg.height * msg.width == port_out.getWidth()
#if DEBUG_EVENT_TRANSFORMATION
	if (msg.events.size() > 0) {
		std::cout << "eventArrayCallback " << msg.events.size() << std::endl;
	}
#endif
	for (int i = 0; i < msg.events.size(); i++) {
		dvs_msgs::Event event = msg.events[i];
		int index = event.y * msg.width + event.x;
		// TODO respect polarity?
#if DEBUG_EVENT_TRANSFORMATION
		std::cout << "event: ts = " << last_tick_time << ", index = " << index << std::endl;
#endif
		port_out->insertEvent(last_tick_time, MUSIC::GlobalIndex(index));
	}
#if BENCHMARK_ROS_CALLBACK
	clock_gettime(CLOCK_MONOTONIC, &finish);
	elapsed = (finish.tv_sec - start.tv_sec);
	elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
	std::cout << "ros_event_sensor_adapter: elapsed time = " << elapsed << std::endl;
#endif
}

void RosEventSensorAdapter::finalize() {
	runtime->finalize();
	delete runtime;
}



