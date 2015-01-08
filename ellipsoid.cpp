#include "ellipsoid.h"

ellipsoid::ellipsoid(Eigen::Vector3f cent, Eigen::Matrix3f covar, Eigen::Vector3f e_val, Eigen::Matrix3f e_vec, Eigen::Vector3f normalizer)
{
    center = cent;
    cov = covar;
    eigen_values = e_val;
    eigen_vectors = e_vec;
    nor = normalizer;
}

ellipsoid::~ellipsoid()
{

}

