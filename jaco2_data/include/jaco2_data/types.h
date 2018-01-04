#ifndef TYPES_H
#define TYPES_H

#include <Eigen/StdVector>
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

typedef std::vector<Vector3,
Eigen::aligned_allocator<Eigen::Vector3d> > Vector3Collection;
typedef std::vector<Vector3Stamped,
Eigen::aligned_allocator<Eigen::Vector3d> > Vector3StampedCollection;

typedef std::vector<JointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> >
JointStateDataCollection;

typedef std::vector<JointStateDataStamped,
Eigen::aligned_allocator<Eigen::Vector3d> >
JointStateDataStampedCollection;

typedef std::deque<JointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > JointStateDataDeque;
typedef std::deque<JointStateDataStamped,
Eigen::aligned_allocator<Eigen::Vector3d> > JointStateDataStampedDeque;


typedef std::vector<ExtendedJointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateCollection;
typedef std::vector<ExtendedJointStateDataStamped,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateStampedCollection;

typedef std::deque<ExtendedJointStateData,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateDeque;
typedef std::deque<ExtendedJointStateDataStamped,
Eigen::aligned_allocator<Eigen::Vector3d> > ExtendedJointStateStampedDeque;

typedef std::vector<JointData> JointDataCollection;
typedef std::vector<JointDataStamped> JointDataStampedCollection;
typedef std::vector<JointAngles> JointAnglesCollection;
typedef std::vector<JointAnglesStamped> JointAnglesStampedCollection;

typedef std::vector<AccelerometerData> AccelerometerDataCollection;
typedef std::deque<AccelerometerData, std::allocator<AccelerometerData>> AccelerometerDataDeque;
}


#endif
