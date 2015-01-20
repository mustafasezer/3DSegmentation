#include "object.h"
#include <cmath>

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
    GMM = cv::EM(7, cv::EM::COV_MAT_DIAGONAL);
    disappeared = false;
    maxVolume = 0;
    ellipsoid_history_size = 10;

    updatePosition(cld, ind);
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

double Object::bhattacharyyaCoeff(cv::EM gmm1, cv::EM gmm2, int formula){
    cv::Mat mean1 = gmm1.getMat("means");
    cv::Mat weight1 = gmm1.getMat("weights");
    std::vector<cv::Mat> covs1 = gmm1.getMatVector("covs");

    cv::Mat mean2 = gmm2.getMat("means");
    cv::Mat weight2 = gmm2.getMat("weights");
    std::vector<cv::Mat> covs2 = gmm2.getMatVector("covs");

    //std::cout << weight1.rows << "x" << weight1.cols << " " << weight2.rows << "x" << weight2.cols << std::endl;

    cv::Mat bhat_dist = cv::Mat(1,1, CV_64F, double(0));
    //std::cout << bhat_dist.at<double>(0,0) << std::endl;


    if(formula==1){
        //Bhattacharyya based calculation as described in the paper
        for(int i=0; i<gmm1.getInt("nclusters"); i++){
            for(int j=0; j<gmm2.getInt("nclusters"); j++){
                cv::Mat COV = covs1[i]+covs2[j];
                //std::cout << covs1[i].rows << "x" << covs1[i].cols << " " << covs2[j].rows << "x" << covs2[j].cols << std::endl;
                cv::Mat meanDiff = mean1.row(i)-mean2.row(j);
                //std::cout << "1x" << mean1.row(i).cols << " and 1x" << mean2.row(j).cols << std::endl;
                bhat_dist += weight1.at<double>(0,i) * weight2.at<double>(0,j) * ( 0.125*meanDiff*(COV.inv()/2)*meanDiff.t() + 0.5*log( cv::determinant(COV/2) / sqrt( cv::determinant(covs1[i])*cv::determinant(covs2[j]) ) ) );
            }
        }
    }
    else if(formula==2){
        //My Battacharyya based calculation
        for(int i=0; i<gmm1.getInt("nclusters"); i++){
            cv::Mat COV = covs1[i]+covs2[i];
            cv::Mat meanDiff = mean1.row(i)-mean2.row(i);
            bhat_dist += weight1.at<double>(0,i) * weight2.at<double>(0,i) * ( 0.125*meanDiff*(COV.inv()/2)*meanDiff.t() + 0.5*log( cv::determinant(COV/2) / sqrt( cv::determinant(covs1[i])*cv::determinant(covs2[i]) ) ) );
        }
    }
    return 1/exp(bhat_dist.at<double>(0,0));
}

double Object::pixelRelation(PointT p){
    return pixelCompatibility(rgb2UV(p))/distance(p);
}

double Object::distance(PointT p){
    Eigen::Vector3f pt(p.x, p.y, p.z);
    return Eigen::Vector3f(e.normCov * (pt-e.center)).norm();
}

double Object::distance(PointT p, ellipsoid ell){
    Eigen::Vector3f pt(p.x, p.y, p.z);
    return Eigen::Vector3f(ell.normCov * (pt-ell.center)).norm();
}

double Object::distance_history(PointT p, int history_index){
    Eigen::Vector3f pt(p.x, p.y, p.z);
    return Eigen::Vector3f(ell_vec[history_index].normCov * (pt-ell_vec[history_index].center)).norm();
}

double Object::pixelCompatibility(cv::Mat point){
    //cv::Mat weights(GMM.getMat("weights"));
    cv::Mat probs(1, GMM.getInt("nclusters"), CV_64FC1);
    GMM.predict(point, probs);
    cv::Scalar compatibility = cv::sum(GMM.getMat("weights").mul(probs));
    return compatibility[0];
}

bool Object::updatePosition(Cloud *cld, pcl::PointIndices ind){

    ellipsoid ell;

    Eigen::Vector4f xyz_centroid;
    computeMeanAndCovarianceMatrix(*cld, ind, ell.cov, xyz_centroid);
    pcl::eigen33(ell.cov, ell.evec, ell.eval);
    ell.axes = K * Eigen::Vector3f(sqrt(ell.eval(0)), sqrt(ell.eval(1)), sqrt(ell.eval(2)));
    ell.center = Eigen::Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
    ell.volume = ell.axes.prod() * M_PI * 4/3;
    ell.normCov = Eigen::Matrix<float, 3, 3>::Identity();
    for(int i=0; i<3; i++){
        ell.normCov(i,i) = 1/ell.axes[i];
    }
    ell.normCov = ell.evec * ell.normCov;

    //TODO History to remove
    //Insert the ellipsoid to the ellipsoid vector
    ell_vec.insert(ell_vec.begin(), ell);
    if(ell_vec.size()>ellipsoid_history_size){
        ell_vec.pop_back();
    }

    if(ell.volume > maxVolume){
        maxVolume = ell.volume;
        e_max = ell;
        e_max_shifted = e_max;
    }
    e_max_shifted.center = ell.center;
    e = ell;

    occlusionRatio = 1-(ell.volume/maxVolume);
    if(occlusionRatio > 0.65){
        occluded = true;
        disappeared = true;
        std::cout << "Object " << id << " is disappeared!" << std::endl;
    }
    else if(occlusionRatio > 0.3){
        occluded = true;
        disappeared = false;
        std::cout << "Object " << id << " is occluded!" << std::endl;
    }
    else{
        occluded = false;
        disappeared = false;
    }

    /*cv::Mat samples(ind.indices.size(), 2, CV_64FC1);
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
    }*/
    //For control purposes
    /*if(GMM.train(samples)){
        cv::Mat p(1, 2, CV_64FC1);
        p.at<double>(0,0) = samples.at<double>(0,0);
        p.at<double>(0,1) = samples.at<double>(0,1);
        double dummy = pixelCompatibility(p);
        cout << dummy << endl;
    }*/
}

bool Object::updateAppearance(Cloud *cld, pcl::PointIndices ind){

    /*Eigen::Vector4f xyz_centroid;
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

    if(e.volume > maxVolume){
        maxVolume = e.volume;
    }
    occlusionRatio = 1-(e.volume/maxVolume);
    if(occlusionRatio > 0.65){
        occluded = true;
        disappeared = true;
        std::cout << "Object " << id << " is disappeared!" << std::endl;
    }
    else if(occlusionRatio > 0.3){
        occluded = true;
        disappeared = false;
        std::cout << "Object " << id << " is occluded!" << std::endl;
    }
    else{
        occluded = false;
        disappeared = false;
    }*/

    //updatePosition(cld, ind);

    int i=0;

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

