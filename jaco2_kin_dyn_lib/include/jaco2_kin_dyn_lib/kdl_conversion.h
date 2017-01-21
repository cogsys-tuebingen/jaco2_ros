#ifndef KDL_CONVERSION_H
#define KDL_CONVERSION_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <Eigen/Core>
#include <tf/tf.h>

namespace Jaco2KinDynLib {
     void convert(const KDL::JntArray& in, std::vector<double>& out);
     void convert(const std::vector<double>& in, KDL::JntArray& out);
     void PoseTFToKDL(const tf::Pose& t, KDL::Frame& k);
     Eigen::Matrix3d skewSymMat(const KDL::Vector& vec);
     Eigen::Matrix<double, 3, 6> inertiaProductMat(const KDL::Vector& vec);
     Eigen::Matrix<double, 6, 6> convert2Eigen(const KDL::Frame& frame);
     Eigen::Matrix<double, 3, 3> convert2Eigen(const KDL::Rotation& rot);
     Eigen::Matrix<double, 6, 1> convert2Eigen(const KDL::Wrench& wrench);
     void kdlJntArray2Eigen(const KDL::JntArray& q, Eigen::VectorXd &res);
     void convert2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd &res);
}
#endif // KDL_CONVERSION_H
