#ifndef KDL_CONVERSION_H
#define KDL_CONVERSION_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <Eigen/Core>
#include <tf/tf.h>

namespace Jaco2KinDynLib {
    static void convert(const KDL::JntArray& in, std::vector<double>& out);
    static void convert(const std::vector<double>& in, KDL::JntArray& out);
    static void PoseTFToKDL(const tf::Pose& t, KDL::Frame& k);
    static Eigen::Matrix3d skewSymMat(const KDL::Vector& vec);
    static Eigen::Matrix<double, 3, 6> inertiaProductMat(const KDL::Vector& vec);
    static Eigen::Matrix<double, 6, 6> kdlFrame2Spatial(const KDL::Frame& frame);
    static Eigen::Matrix<double, 3, 3> kdlMatrix2Eigen(const KDL::Rotation& rot);
    static void kdlJntArray2Eigen(const KDL::JntArray& q, Eigen::VectorXd &res);
    static void kdlMatrix2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd &res);
}
#endif // KDL_CONVERSION_H
