#ifndef ACCELEROMETER_DATA_H
#define ACCELEROMETER_DATA_H
#include <jaco2_data/vector3stamped.h>
namespace jaco2_data {

class AccelerometerData
{
public:
    typedef std::vector<Vector3Stamped>::iterator iterator;
    typedef std::vector<Vector3Stamped>::const_iterator const_iterator;
public:

    AccelerometerData();

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    Vector3Stamped& at(std::size_t i);
    const Vector3Stamped& at(std::size_t i) const;

    Vector3Stamped& operator[](std::size_t i);
    const Vector3Stamped& operator [](std::size_t i) const;

    std::size_t size() const;
    void resize(std::size_t n, Vector3Stamped val = Vector3Stamped(0,0,0));

    void emplace_back(Vector3Stamped&& val);
    void push_back(const Vector3Stamped& val);

public:
    int user_defined_label;
private:
    std::vector<Vector3Stamped> lin_acc;
};
}
#endif // ACCELEROMETER_DATA_H
