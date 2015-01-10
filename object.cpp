#include "object.h"

#include <pcl/features/feature.h>     //For eigen33
//Includes for ellipsoid calculations
#include <pcl/common/centroid.h>
#include <pcl/ros/conversions.h>

//#include <opencv2/core/eigen.hpp>     //for eigen2cv


float K = 2.795483482915108;        // 95% confidence


void outputMatrix(cv::Mat m){
    for(int i=0; i<m.rows; i++){
        for(int j=0; j<m.cols; j++){
            std::cout << m.at<double>(i,j) << " ";
        }
        std::cout << std::endl;
    }
}


/*Object::Object(int id_, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f axes)
{
    id = id_;
    e.center = center;
    e.cov = cov;
    e.eval = eval;
    e.evec = evec;
    e.axes = axes;
    e.volume = axes.prod() * M_PI * 4/3;
    e.normCov = Eigen::Matrix<float, 3, 3>::Identity();
    for(int i=0; i<3; i++){
        e.normCov(i,i) = 1/axes[i];
    }
    e.normCov = e.evec * e.normCov;

    //cv::eigen2cv(normCov, e.normCov);
    //outputMatrix(e.normCov);

    GMM = cv::EM(5, cv::EM::COV_MAT_DIAGONAL);
    disappeared = false;
}*/


Object::Object(int id_, Cloud *cld, pcl::PointIndices ind)
{
    id = id_;
    //cloud = cld;
    GMM = cv::EM(5, cv::EM::COV_MAT_DIAGONAL);
    disappeared = false;

    updateAppearance(cld, ind);

    //cv::eigen2cv(normCov, e.normCov);
    //outputMatrix(e.normCov);


}

Object::~Object()
{

}

cv::Mat Object::rgb2UV(PointT point){
    //CV_RGB2YCrCb
    cv::Mat p(1, 2, CV_64FC1);
    p.at<double>(0,0) = -0.147*point.r - 0.289*point.g + 0.436*point.b;
    p.at<double>(0,1) =  0.615*point.r - 0.515*point.g - 0.100*point.b;
    return p;
}

double Object::distance(PointT p){
    Eigen::Vector3f pt(p.x, p.y, p.z);
    return Eigen::Vector3f(e.normCov * (pt-e.center)).norm();
}

double Object::pixelCompatibility(cv::Mat point){
    //cv::Mat weights(GMM.getMat("weights"));
    cv::Mat probs(1, GMM.getInt("nclusters"), CV_64FC1);
    GMM.predict(point, probs);
    cv::Scalar compatibility = cv::sum(GMM.getMat("weights").mul(probs));
    return compatibility[0];
}

bool Object::updateAppearance(Cloud *cld, pcl::PointIndices ind){
    int i=0;

    Eigen::Vector4f xyz_centroid;
    computeMeanAndCovarianceMatrix(*cld, ind, e.cov, xyz_centroid);
    pcl::eigen33(e.cov, e.evec, e.eval);
    e.axes = K * Eigen::Vector3f(sqrt(e.eval(0)), sqrt(e.eval(1)), sqrt(e.eval(2)));
    e.center = Eigen::Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
    e.volume = e.axes.prod() * M_PI * 4/3;
    e.normCov = Eigen::Matrix<float, 3, 3>::Identity();
    for(int i=0; i<3; i++){
        e.normCov(i,i) = 1/e.axes[i];
    }
    e.normCov = e.evec * e.normCov;

    cv::Mat samples(ind.indices.size(), 2, CV_64FC1);
    for (std::vector<int>::const_iterator pit = ind.indices.begin (); pit != ind.indices.end (); pit++){
        cv::Mat point = rgb2UV(cld->points[*pit]);
        samples.at<double>(i,0) = point.at<double>(0,0);
        samples.at<double>(i,1) = point.at<double>(0,1);
        i++;
    }
    if(GMM.isTrained()){
        return GMM.trainE(samples, GMM.getMat("means"), GMM.getMatVector("covs"), GMM.getMat("weights"));
    }
    else{
        return GMM.train(samples);
    }
    //For control purposes
    /*if(GMM.train(samples)){
        cv::Mat p(1, 2, CV_64FC1);
        p.at<double>(0,0) = samples.at<double>(0,0);
        p.at<double>(0,1) = samples.at<double>(0,1);
        double dummy = pixelCompatibility(p);
        cout << dummy << endl;
    }*/
}

/*bool Object::updateObject(int id_, Eigen::Vector3f center, Eigen::Matrix3f cov, Eigen::Vector3f eval, Eigen::Matrix3f evec, Eigen::Vector3f axes)
{
    id = id_;
    e.center = center;
    e.cov = cov;
    e.eval = eval;
    e.evec = evec;
    e.axes = axes;
    e.volume = axes.prod() * M_PI * 4/3;
    e.normCov = Eigen::Matrix<float, 3, 3>::Identity();
    for(int i=0; i<3; i++){
        e.normCov(i,i) = 1/axes[i];
    }
    e.normCov = e.evec * e.normCov;
    disappeared = false;
}*/

/*void Object::deneme(CloudPtr cloud, pcl::PointIndices ind){
    cv::EM myGMM = cv::EM(5, cv::EM::COV_MAT_DIAGONAL);
    int i=0;
    cv::Mat samples(5, 2, CV_64FC1);
    for (std::vector<int>::const_iterator pit = ind.indices.begin (); pit != ind.indices.end (); pit++){
        cv::Mat point = rgb2UV(cloud->points[*pit]);
        samples.at<double>(i,0) = point.at<double>(0,0);
        samples.at<double>(i,1) = point.at<double>(0,1);
        i++;
        if(i==5)
            break;
    }

    myGMM.train(samples);

    if(myGMM.isTrained()){
        cv::Mat probs(1, myGMM.getInt("nclusters"), CV_64FC1);
        cv::Mat toplam = cv::Mat::zeros(1, 2, CV_64FC1);
        for(int j=0; j<5; j++){
            toplam += samples.row(j);
            myGMM.predict(samples.row(j), probs);
            outputMatrix(probs);
        }
        toplam = toplam/5;
        myGMM.predict(toplam, probs);
        outputMatrix(probs);
        toplam = (samples.row(3)+samples.row(4))/2;
        myGMM.predict(toplam, probs);
        outputMatrix(probs);
        cv::Scalar compatibility = cv::sum(myGMM.getMat("weights").mul(probs));
        std::cout << compatibility[0] << std::endl;
    }

}*/

