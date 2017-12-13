#ifndef HEADER_H
#define HEADER_H

#include <string>
#include <jaco2_data/time_stamp.h>
namespace jaco2_data
{

struct Header{
    TimeStamp stamp;
    std::string frame_id;
};
}
#endif // HEADER_H
