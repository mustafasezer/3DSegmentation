#ifndef OBJECT_H
#define OBJECT_H

//#include "ellipsoid.h"

#include "pcssegmentation.h"
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

struct ellipsoidStruct{
    Eigen::Vector3f center;
    Eigen::Matrix3f cov;
    Eigen::Vector3f eval;
    Eigen::Matrix3f evec;
    Eigen::Vector3f nor;        //Normalizer
    float volume;
};


typedef struct ellipsoidStruct ellipsoid;

class Object
{
public:
    Object(int id, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f nor);
    ~Object();
    bool trainAppearance(CloudPtr cloud, pcl::PointIndices ind);
    cv::Mat rgb2UV(PointT point);
    double pixelCompatibility(cv::Mat point);
    int id;
    ellipsoid e;
    cv::EM GMM;
    bool is1to1;
    bool isoccluded;
    float maxVolume;
    //blob
};

#endif // OBJECT_H
