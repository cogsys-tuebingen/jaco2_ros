#ifndef STAMPED_DATA_H
#define STAMPED_DATA_H
#include <jaco2_data/header.h>
namespace jaco2_data{
template<typename T>
class StampedData
{
public:
    StampedData() {}

    StampedData(T d) :
        data(d)
    {}

    StampedData(T& d) :
        data(d)
    {}

    StampedData(jaco2_data::Header& h, T& d) :
        header(h),
        data(d)
    {}

    TimeStamp stamp() const
    {
        return header.stamp;
    }
    TimeStamp& stamp()
    {
        return header.stamp;
    }

    std::string frameId() const
    {
        return header.frame_id;
    }

    std::string& frameId()
    {
        return header.frame_id;
    }

    StampedData<T> operator+(const StampedData<T> &other) const
    {
        StampedData<T> res;
        res.header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() + other.header.stamp.toNSec()));
        res.header.frame_id = other.header.frame_id;
        res.data = this->data + other.data;
        return res;
    }

    StampedData<T> operator-(const StampedData<T> &other) const
    {
        StampedData<T> res;
        res.header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() - other.header.stamp.toNSec()));
        res.header.frame_id = other.header.frame_id;
        res.data = this->data - other.data;
        return res;
    }

    StampedData<T> operator*(const double& b) const
    {
        StampedData<T> res;
        res.header  = header;
        res.data = this->data / b;
        return res;
    }

    StampedData<T> operator/(const double& b) const
    {
        StampedData<T> res;
        res.header  = header;
        res.data = this->data / b;
        return res;
    }

    StampedData<T>& operator+=(const StampedData<T> &other)
    {
        this->header.stamp.fromNSec(0.5* (this->header.stamp.toNSec() + other.header.stamp.toNSec()));
        this->data += other.data;
        return *this;
    }

    StampedData<T>& operator*=(const double &b)
    {
        this->data *= b;
    }

    StampedData<T>& operator/=(const double &b)
    {
        this->data /= b;
    }

    std::string toString(std::string delimiter = std::string(";")) const
    {
        std::stringstream ss;
        ss << std::to_string(header.stamp.toNSec()) + delimiter;
        ss << data.toString(delimiter);
        return ss.str();
    }

public:
    Header header;
    T data;
};
}
#endif // STAMPED_DATA_H
