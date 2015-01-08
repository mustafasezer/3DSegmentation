#include "object.h"

Object::Object(int id, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f nor)
{
    e.center = center;
    e.cov = cov;
    e.eval = eval;
    e.evec = evec;
    e.nor = nor;
    e.volume = nor.prod();// * M_PI * 4/3;
    GMM = cv::EM(10, cv::EM::COV_MAT_DIAGONAL);
}

Object::~Object()
{
    //CV_RGB2YCrCb
}

cv::Mat Object::rgb2UV(PointT point){
    cv::Mat p(1, 2, CV_64FC1);
    p.at<double>(0,0) = -0.147*point.r - 0.289*point.g + 0.436*point.b;
    p.at<double>(0,1) =  0.615*point.r - 0.515*point.g - 0.100*point.b;
    return p;
}

double Object::pixelCompatibility(cv::Mat point){
    cv::Mat weights(GMM.getMat("weights"));
    cv::Mat probs(1, GMM.getInt("nclusters"), CV_64FC1);
    GMM.predict(point, probs);
    cv::Scalar compatibility = cv::sum(GMM.getMat("weights").mul(probs));
    return compatibility[0];
}

bool Object::trainAppearance(CloudPtr cloud, pcl::PointIndices ind){
    int i=0;
    double u, v;
    cv::Mat samples(ind.indices.size(), 2, CV_64FC1);
    for (std::vector<int>::const_iterator pit = ind.indices.begin (); pit != ind.indices.end (); pit++){
        cv::Mat point = rgb2UV(cloud->points[*pit]);
        samples.at<double>(i,0) = point.at<double>(0,0);
        samples.at<double>(i,1) = point.at<double>(0,1);
        i++;
    }
    return GMM.train(samples);



    //For control purposes
    /*if(GMM.train(samples)){
        cv::Mat p(1, 2, CV_64FC1);
        p.at<double>(0,0) = samples.at<double>(0,0);
        p.at<double>(0,1) = samples.at<double>(0,1);
        double dummy = pixelCompatibility(p);
        cout << dummy << endl;
    }*/

}

