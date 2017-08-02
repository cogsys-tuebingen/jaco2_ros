#ifndef ACCELEROMETER_DATA_H
#define ACCELEROMETER_DATA_H
#include <vector>
#include <Eigen/StdVector>
#include <jaco2_data/vector3stamped.h>
namespace jaco2_data {

class AccelerometerData
{
public:
    typedef Eigen::aligned_allocator<Eigen::Vector3d> vector_3_stamped_aligned_allocator ;
    typedef std::vector<Vector3Stamped, vector_3_stamped_aligned_allocator>::iterator iterator;
    typedef std::vector<Vector3Stamped, vector_3_stamped_aligned_allocator>::const_iterator const_iterator;
public:

    AccelerometerData();
    AccelerometerData(std::size_t n);

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    Vector3Stamped& at(std::size_t i);
    const Vector3Stamped& at(std::size_t i) const;

    Vector3Stamped& operator[](std::size_t i);
    const Vector3Stamped& operator [](std::size_t i) const;

    Vector3Stamped& front();
    const Vector3Stamped& front() const;

    Vector3Stamped& back();
    const Vector3Stamped& back() const;

    std::size_t size() const;
    void resize(std::size_t n, Vector3Stamped val = Vector3Stamped(0,0,0));

//    void emplace_back(Vector3Stamped&& val);
    void push_back(const Vector3Stamped& val);

    std::vector<double> toVector() const;
    AccelerometerData abs() const;

    double norm() const;

    AccelerometerData operator+(const AccelerometerData &other) const;
    AccelerometerData& operator+=(const AccelerometerData &other);
    AccelerometerData& operator*=(const double &b);
    AccelerometerData& operator/=(const double &b);


public:
    int label;
private:
    std::vector<Vector3Stamped, vector_3_stamped_aligned_allocator> lin_acc;
};
}
#endif // ACCELEROMETER_DATA_H
