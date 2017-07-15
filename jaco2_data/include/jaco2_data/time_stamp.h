#ifndef TIME_STAMP_H
#define TIME_STAMP_H
#include <chrono>
namespace jaco2_data {

class TimeStamp
{
public:
    TimeStamp();

    void now();

    unsigned long int toNSec() const;
    unsigned long int toMicroSec() const;
    double toSec() const;

    void fromNSec(unsigned long int nsecs);

    bool operator !=(const TimeStamp& other) const;

    double substractionResultInSeconds(const TimeStamp& lhs) const;


public:
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
};

}
#endif // TIME_STAMP_H
