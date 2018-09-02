#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <Eigen/Core>
#include <jaco2_data/stamped_data.h>

// FORWARD DECLARATIONS

namespace std
{
template <typename _Tp, typename _Alloc>
class deque;
}

namespace jaco2_data
{
class JointStateData;
class Vector3;
class Wrench;
class ExtendedJointStateData;
class AccelerometerData;
class JointData;
class JointAngles;
}


// TYPEDEFS
typedef Eigen::aligned_allocator<Eigen::Vector3d> EV3dAllocator;

namespace jaco2_data
{
typedef StampedData<JointStateData> JointStateDataStamped;
typedef StampedData<Vector3> Vector3Stamped;
typedef StampedData<Wrench> WrenchStamped;
typedef StampedData<ExtendedJointStateData> ExtendedJointStateDataStamped;
typedef StampedData<JointData> JointDataStamped;
typedef StampedData<JointAngles> JointAnglesStamped;

typedef std::vector<Vector3, EV3dAllocator> Vector3Collection;
typedef std::vector<Vector3Stamped, EV3dAllocator> Vector3StampedCollection;

typedef std::vector<JointStateData, EV3dAllocator> JointStateDataCollection;

typedef std::vector<JointStateDataStamped, EV3dAllocator> JointStateDataStampedCollection;

typedef std::deque<JointStateData, EV3dAllocator> JointStateDataDeque;
typedef std::deque<JointStateDataStamped, EV3dAllocator> JointStateDataStampedDeque;


typedef std::vector<ExtendedJointStateData, EV3dAllocator> ExtendedJointStateCollection;
typedef std::vector<ExtendedJointStateDataStamped, EV3dAllocator> ExtendedJointStateStampedCollection;

typedef std::deque<ExtendedJointStateData, EV3dAllocator> ExtendedJointStateDeque;
typedef std::deque<ExtendedJointStateDataStamped, EV3dAllocator> ExtendedJointStateStampedDeque;

typedef std::vector<JointData> JointDataCollection;
typedef std::vector<JointDataStamped> JointDataStampedCollection;
typedef std::vector<JointAngles> JointAnglesCollection;
typedef std::vector<JointAnglesStamped> JointAnglesStampedCollection;

typedef std::vector<AccelerometerData> AccelerometerDataCollection;
typedef std::deque<AccelerometerData, std::allocator<AccelerometerData>> AccelerometerDataDeque;
}


#endif
