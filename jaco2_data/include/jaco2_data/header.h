#ifndef HEADER_H
#define HEADER_H

#include <string>
#include <jaco2_data/time_stamp.h>
namespace jaco2_data
{

struct Header{

    Header() {}
    Header(const Header& h):
        stamp(h.stamp),
        frame_id(h.frame_id)
    {}

    Header(Header && h):
        stamp(h.stamp),
        frame_id(std::move(h.frame_id))
    {
    }

    Header(const std::string& frame_id_, const TimeStamp& stamp_) :
        stamp(stamp_),
        frame_id(frame_id_)
    {
    }

    Header& operator = (const Header &h)
    {
        stamp = h.stamp;
        frame_id = h.frame_id;
        return *this;
    }

    Header& operator = (Header &&h)
    {
        stamp = h.stamp;
        frame_id = std::move(h.frame_id);
        return *this;
    }



    TimeStamp stamp;
    std::string frame_id;
};
}
#endif // HEADER_H
