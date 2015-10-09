#ifndef RATE_H
#define RATE_H

#include "boost/thread.hpp"
#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o

using namespace boost::posix_time;
class Rate
{
    public:
        time_duration last_time_to_sleep;

        Rate(double rate)
        {
            timestep = microseconds(1000000./rate);
            zero = microseconds(0);
            t0 = ptime(microsec_clock::universal_time());
            dt = microseconds(0);
            last_time_to_sleep = microseconds(0);

        }

        void sleep()
        {
            t1 = ptime(microsec_clock::universal_time());
            dt = t1 - t0;
            time_to_sleep = timestep - dt;
            if (last_time_to_sleep < zero)
            {
                time_to_sleep += last_time_to_sleep;
            } 
            if (time_to_sleep > zero)
            {
                boost::this_thread::sleep(time_to_sleep);
            }
            last_time_to_sleep = time_to_sleep;
            t0 = ptime(microsec_clock::universal_time());
            
        }
    private:
        time_duration timestep; 
        ptime t0;
        ptime t1;

        time_duration zero;
        time_duration time_to_sleep; 
        time_duration dt;
};
#endif
