#include <jaco2_data/time_stamp.h>

using namespace jaco2_data;


TimeStamp::TimeStamp() {}

void TimeStamp::now()
{
    stamp = std::chrono::high_resolution_clock::now();
}

unsigned long int TimeStamp::toNSec() const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
}

unsigned long int TimeStamp::toMicroSec() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
}

double TimeStamp::toSec() const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count() * 1e-9;
}

void TimeStamp::fromNSec(unsigned long int nsecs)
{
    std::chrono::nanoseconds  tmp = std::chrono::nanoseconds(nsecs);
    stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
}

void TimeStamp::fromMicroSec(unsigned long int musecs)
{
    std::chrono::microseconds  tmp = std::chrono::microseconds(musecs);
    stamp = std::chrono::time_point<std::chrono::high_resolution_clock>(tmp);
}

bool TimeStamp::operator !=(const TimeStamp& other) const
{
    return stamp != other.stamp;
}

bool TimeStamp::operator <=(const TimeStamp& other) const
{
    return stamp <= other.stamp;
}

bool TimeStamp::operator ==(const TimeStamp& other) const
{
    return stamp == other.stamp;
}

double TimeStamp::substractionResultInSeconds(const TimeStamp &lhs) const
{
    auto delta = stamp - lhs.stamp;
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(delta).count()*1e-6;
    return dt;
}

double TimeStamp::timeDiffinSeconds(const TimeStamp &lhs, const TimeStamp &rhs)
{
    return lhs.substractionResultInSeconds(rhs);
}
