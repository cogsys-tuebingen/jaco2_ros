#ifndef DISTRIBUTION_HPP
#define DISTRIBUTION_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace jaco2_utils {
namespace math {
namespace statistic {
template<bool limit_covariance = false>
class Distribution {
public:
    typedef std::shared_ptr<Distribution<limit_covariance>> Ptr;

    typedef Eigen::VectorXd       PointType;
    typedef Eigen::MatrixXd       MatrixType;
    typedef Eigen::MatrixXd       EigenValueSetType;
    typedef Eigen::MatrixXd       EigenVectorSetType;
    typedef Eigen::VectorXcd      ComplexVectorType;
    typedef Eigen::MatrixXcd      ComplexMatrixType;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);
    static constexpr double lambda_ratio = 1e-2;

    Distribution(std::size_t dim = 1) :
        dim_(dim),
        mean(PointType::Zero(dim_)),
        correlated(MatrixType::Zero(dim_, dim_)),
        n(1),
        n_1(0),
        covariance(MatrixType::Zero(dim_, dim_)),
        inverse_covariance(MatrixType::Zero(dim_, dim_)),
        eigen_values(EigenValueSetType::Zero(dim_, dim_)),
        eigen_vectors(EigenVectorSetType::Zero(dim_, dim_)),
        determinant(0.0),
        dirty(false),
        dirty_eigen(false)
    {
    }

    Distribution(const Distribution &other) = default;
    Distribution& operator=(const Distribution &other) = default;

    inline void reset()
    {
        mean = PointType::Zero(dim_);
        covariance = MatrixType::Zero(dim_, dim_);
        correlated = MatrixType::Zero(dim_, dim_);
        n = 1;
        n_1 = 0;
        dirty = true;
        dirty_eigen = true;
    }

    /// Modification
    inline void add(const PointType &_p)
    {
        mean = (mean * n_1 + _p) / n;
        for(std::size_t i = 0 ; i < dim_ ; ++i) {
            for(std::size_t j = i ; j < dim_ ; ++j) {
                correlated(i, j) = (correlated(i, j) * n_1 + _p(i) * _p(j)) / (double) n;
            }
        }
        ++n;
        ++n_1;
        dirty = true;
        dirty_eigen = true;
    }

    inline Distribution& operator+=(const PointType &_p)
    {
        add(_p);
        return *this;
    }

    inline Distribution& operator+=(const Distribution &other)
    {
        std::size_t _n = n_1 + other.n_1;
        PointType   _mean = (mean * n_1 + other.mean * other.n_1) / (double) _n;
        MatrixType  _corr = (correlated * n_1 + other.correlated * other.n_1) / (double) _n;
        n   = _n + 1;
        n_1 = _n;
        mean = _mean;
        correlated = _corr;
        dirty = true;
        dirty_eigen = true;
        return *this;
    }

    /// Distribution properties
    inline std::size_t getN() const
    {
        return n_1;
    }

    inline PointType getMean() const
    {
        return mean;
    }

    inline void getMean(PointType &_mean) const
    {
        _mean = mean;
    }

    inline MatrixType getCovariance() const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            return covariance;
        }
        return MatrixType::Zero(dim_, dim_);
    }

    inline void getCovariance(MatrixType &_covariance) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            _covariance = covariance;
        } else {
            _covariance = MatrixType::Zero();
        }
    }

    inline MatrixType getInformationMatrix() const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            return inverse_covariance;
        }
        return MatrixType::Zero(dim_, dim_);
    }

    inline void getInformationMatrix(MatrixType &_inverse_covariance) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            _inverse_covariance = inverse_covariance;
        } else {
            _inverse_covariance = MatrixType::Zero(dim_, dim_);
        }
    }

    inline EigenValueSetType getEigenValues(const bool _abs = false) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            if(_abs)
                return eigen_values.cwiseAbs();
            else
                return eigen_values;
        }
        return EigenValueSetType::Zero(dim_, dim_);
    }

    inline void getEigenValues(EigenValueSetType &_eigen_values,
                               const double _abs = false) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            if(_abs)
                _eigen_values = eigen_values.cwiseAbs();
            else
                _eigen_values = eigen_values;
        } else {
            _eigen_values = EigenValueSetType::Zero(dim_, dim_);
        }
    }

    inline EigenVectorSetType getEigenVectors() const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            return eigen_vectors;
        }
        return EigenVectorSetType::Zero();
    }

    inline void getEigenVectors(EigenVectorSetType &_eigen_vectors) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            _eigen_vectors = eigen_vectors;
        } else {
            _eigen_vectors = EigenVectorSetType::Zero(dim_, dim_);
        }
    }

    /// Evaluation
    inline double sample(const PointType &_p) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            PointType  q = _p - mean;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance * q);
            double denominator = 1.0 / (covariance.determinant() * sqrt_2_M_PI);
            return denominator * exp(exponent);
        }
        return 0.0;
    }

    inline double sample(const PointType &_p,
                         PointType &_q) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            _q = _p - mean;
            double exponent = -0.5 * double(_q.transpose() * inverse_covariance * _q);
            double denominator = 1.0 / (determinant * sqrt_2_M_PI);
            return denominator * exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &_p) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();

            PointType  q = _p - mean;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance * q);
            return exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &_p,
                                      PointType &_q) const
    {
        if(n_1 >= 2) {
            if(dirty)
                update();
            _q = _p - mean;
            double exponent = -0.5 * double(_q.transpose() * inverse_covariance * _q);
            return exp(exponent);
        }
        return 0.0;
    }

private:
    const std::size_t            dim_;
    PointType                    mean;
    MatrixType                   correlated;
    std::size_t                  n;
    std::size_t                  n_1;            /// actual amount of points in distribution

    mutable MatrixType           covariance;
    mutable MatrixType           inverse_covariance;
    mutable EigenValueSetType    eigen_values;
    mutable EigenVectorSetType   eigen_vectors;
    mutable double               determinant;

    mutable bool                 dirty;
    mutable bool                 dirty_eigen;

    inline void update() const
    {
        double scale = n_1 / (double)(n_1 - 1);
        for(std::size_t i = 0 ; i < dim_ ; ++i) {
            for(std::size_t j = i ; j < dim_ ; ++j) {
                covariance(i, j) = (correlated(i, j) - (mean(i) * mean(j))) * scale;
                covariance(j, i) = covariance(i, j);
            }
        }

        if(limit_covariance) {
            if(dirty_eigen)
                updateEigen();

            double max_lambda = std::numeric_limits<double>::lowest();
            for(std::size_t i = 0 ; i < dim_ ; ++i) {
                if(eigen_values(i) > max_lambda)
                    max_lambda = eigen_values(i);
            }
            MatrixType Lambda = MatrixType::Zero(dim_, dim_);
            double l = max_lambda * lambda_ratio;
            for(std::size_t i = 0 ; i < dim_; ++i) {
                if(fabs(eigen_values(i)) < fabs(l)) {
                    Lambda(i,i) = l;
                } else {
                    Lambda(i,i) = eigen_values(i);
                }
            }
            covariance = eigen_vectors * Lambda * eigen_vectors.transpose();
            inverse_covariance = eigen_vectors * Lambda.inverse() * eigen_vectors.transpose();
        } else {
            inverse_covariance = covariance.inverse();
        }

        determinant = covariance.determinant();
        dirty = false;
        dirty_eigen = true;
    }

    inline void updateEigen() const
    {
        Eigen::EigenSolver<MatrixType> solver;
        solver.compute(covariance);
        eigen_vectors = solver.eigenvectors().real();
        eigen_values  = solver.eigenvalues().real();
        dirty_eigen = false;
    }
};
}
}
}

#endif /* DISTRIBUTION_HPP */
