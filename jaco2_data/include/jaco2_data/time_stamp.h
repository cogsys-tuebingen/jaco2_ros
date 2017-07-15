#ifndef TIME_STAMP_H
#define TIME_STAMP_H
#include <chrono>
namespace jaco2_data {

class TimeStamp
{
public:
    TimeStamp() {}

    inline void now()
    {
        stamp = std::chrono::high_resolution_clock::now();
    }

    inline unsigned long int toNSec() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
    }

    inline unsigned long int toMicroSec() const
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
    }

    inline double toSec() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count() * 1e-9;
    }

    inline void fromNSec(unsigned long int nsecs)
    {
        std::chrono::nanoseconds  tmp = std::chrono::nanoseconds(nsecs);
        stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
    }


public:
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
};

}
#endif // TIME_STAMP_H
