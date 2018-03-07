#ifndef HEADER_H
#define HEADER_H

#include <string>
#include <jaco2_data/time_stamp.h>
namespace jaco2_data
{

struct Header{

    Header() {}

    Header(const std::string& frame_id_, const TimeStamp& stamp_) :
        stamp(stamp_),
        frame_id(frame_id_)
    {
    }

    TimeStamp stamp;
    std::string frame_id;
};
}
#endif // HEADER_H
