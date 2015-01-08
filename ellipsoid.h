#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <Eigen/Core>
//#include <Eigen/Eigenvalues><


class ellipsoid
{
public:
    ellipsoid(Eigen::Vector3f cent, Eigen::Matrix3f covar, Eigen::Vector3f e_val, Eigen::Matrix3f e_vec, Eigen::Vector3f normalizer);
    ~ellipsoid();
    Eigen::Vector3f center;
    Eigen::Matrix3f cov;
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;
    Eigen::Vector3f nor;        //Normalizer
};

#endif // ELLIPSOID_H
