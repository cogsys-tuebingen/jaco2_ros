#ifndef TYPES_H
#define TYPES_H

#include <Eigen/StdVector>

// FORWARD DECLARATIONS

namespace std
{
template <typename _Tp, typename _Alloc>
class deque;
}

namespace jaco2_data
{
class JointStateData;
class Vector3Stamped;
class ExtendedJointStateData;
class AccelerometerData;
class JointData;
class JointAngles;
}


// TYPEDEFS
typedef Eigen::aligned_allocator<Eigen::Vector3d> EV3dAllocator;

namespace jaco2_data
{
typedef std::vector<Vector3Stamped,
Eigen::aligned_allocator<Eigen::Vector3d> > Vector3StampedCollection;

typedef std::vector<JointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> >
JointStateDataCollection;

typedef std::deque<JointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > JointStateDataDeque;

typedef std::vector<ExtendedJointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateCollection;

typedef std::deque<ExtendedJointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateDeque;

typedef std::vector<JointData> JointDataCollection;
typedef std::vector<JointAngles> JointAnglesCollection;

typedef std::vector<AccelerometerData> AccelerometerDataCollection;
typedef std::deque<AccelerometerData, std::allocator<AccelerometerData>> AccelerometerDataDeque;
}


#endif