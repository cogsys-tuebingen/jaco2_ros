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
    void fromMicroSec(unsigned long int musecs);
    void fromSec(double sec);

    bool operator !=(const TimeStamp& other) const;
    bool operator <=(const TimeStamp& other) const;
    bool operator >=(const TimeStamp& other) const;
    bool operator ==(const TimeStamp& other) const;

    double substractionResultInSeconds(const TimeStamp& rhs) const;

    static double timeDiffinSeconds(const TimeStamp& lhs, const TimeStamp& rhs);


public:
    std::chrono::time_point<std::chrono::high_resolution_clock> stamp;
};

}
#endif // TIME_STAMP_H
