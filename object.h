#ifndef OBJECT_H
#define OBJECT_H

//#include "ellipsoid.h"

//#include "pcssegmentation.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <Eigen/Core>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

struct ellipsoidStruct{
    Eigen::Vector3f center;
    Eigen::Matrix3f cov;
    Eigen::Vector3f eval;
    Eigen::Matrix3f evec;
    Eigen::Vector3f axes;
    Eigen::Matrix3f normCov;
    //cv::Mat<3,3,CV_64FC1> normCov;        //Normalizer
    float volume;
};


typedef struct ellipsoidStruct ellipsoid;

class Object
{
public:
    //Object(int id_, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f axes);
    Object(int id_, Cloud *cld, pcl::PointIndices ind);
    ~Object();
    //bool updateObject(int id_, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f axes);
    bool updateAppearance(Cloud *cld, pcl::PointIndices ind);
    static cv::Mat rgb2UV(PointT point);
    double pixelCompatibility(cv::Mat point);
    double distance(PointT p);
    int id;
    ellipsoid e;
    cv::EM GMM;
    float maxVolume;
    int blobID;
    bool disappeared;
    bool is1to1;
    bool isoccluded;

    CloudPtr cloud;

    //blob
    //void deneme(CloudPtr cld, pcl::PointIndices ind);
};

#endif // OBJECT_H
