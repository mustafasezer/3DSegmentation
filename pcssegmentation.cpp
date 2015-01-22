#include "pcssegmentation.h"
#include <iostream>
#include <fstream>

/*#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>*/

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/min_cut_segmentation.h>

//Includes for region growing
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include "object.h"

using namespace pcl;
using namespace Eigen;

std::vector<Object> objects;


PCSSegmentation::PCSSegmentation()
{
    maxID = 100;

    srand(time(NULL));
    r = new int[maxID];
    g = new int[maxID];
    b = new int[maxID];

    int rr[8] = {255, 0, 127, 127, 255, 0, 0, 255};
    int gg[8] = {0, 255, 255, 0, 191, 255, 63, 0};
    int bb[8] = {0, 255, 0, 255, 0, 63, 255, 191};

    for(int i=0;i<maxID;i++){
        if(i<8){
            r[i] = rr[i];
            g[i] = gg[i];
            b[i] = bb[i];
        }
        else{
            r[i] = rand() % 256;
            g[i] = rand() % 256;
            b[i] = rand() % 256;
        }
    }

    /*cv::Mat hsvcolors(8, 1, CV_8UC3);
    hsvcolors.at<cv::Vec3b>(0, 0)[0] = 0;
    hsvcolors.at<cv::Vec3b>(1, 0)[0] = 90;
    hsvcolors.at<cv::Vec3b>(2, 0)[0] = 135;
    hsvcolors.at<cv::Vec3b>(3, 0)[0] = 45;
    hsvcolors.at<cv::Vec3b>(4, 0)[0] = 23;
    hsvcolors.at<cv::Vec3b>(5, 0)[0] = 68;
    hsvcolors.at<cv::Vec3b>(6, 0)[0] = 113;
    hsvcolors.at<cv::Vec3b>(7, 0)[0] = 158;

    for (int i =0; i <hsvcolors.cols; i++){
        for (int j =0; j <hsvcolors.rows; j++) {
            hsvcolors.at<cv::Vec3b>(j, i)[1] = 255;
            hsvcolors.at<cv::Vec3b>(j, i)[1] = 255;
            for(int k = 0; k <image.channels(); k++) {
                uchar col = intensity.val[k];
            }
        }
    }
    cv::Mat bgrcolors;
    cv::cvtColor(hsvcolors, bgrcolors, CV_HSV2BGR);
    for(int i=0;i<8;i++){
        r[i] = bgrcolors.at<cv::Vec3b>(i,0)[2];
        g[i] = bgrcolors.at<cv::Vec3b>(i,0)[1];
        b[i] = bgrcolors.at<cv::Vec3b>(i,0)[0];
    }
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", bgrcolors);
    cv::waitKey();*/

}

/*ellipse PCSSegmentation::createEllipse(Cloud incloud, pcl::PointIndices ind){
    ellipse e;
    Eigen::Vector4f xyz_centroid;
    //pcl::compute3DCentroid (incloud, *it, xyz_centroid);
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    //computeCovarianceMatrixNormalized (incloud, xyz_centroid, covariance_matrix);
    // OLD computeMeanAndCovarianceMatrix(incloud, *it, e.cov, xyz_centroid);
    computeMeanAndCovarianceMatrix(incloud, ind, e.cov, xyz_centroid);
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33(e.cov, e.eigen_vectors, e.eigen_values);

    e.center = Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
    return e;
}*/


/*bool PCSSegmentation::isInterior(PointT p, ellipse e){
    return true;
    float dist = e.eigen_vectors * Eigen::Vector3f((p.x-e.center[0]))

            norm(vecm*[(px-centm(1))/x_norm; (py-centm(2))/y_norm; (pz-centm(3))/z_norm]);
}*/

void PCSSegmentation::showObject(int id, CloudPtr pc_input, CloudPtr pc_output){
    for(int i=0; i<objects[id].pointIndices.size(); i++){
        PointT point = pc_input->points[objects[id].pointIndices[i]];
        point.r = r[id];
        point.g = g[id];
        point.b = b[id];
        pc_output->points.push_back(point);
    }
}

void PCSSegmentation::visualizeObjectCenter(CloudPtr obj_cloud, Eigen::Vector3f center){
    PointT centpt;
    centpt.x = center[0];
    centpt.y = center[1];
    centpt.z = center[2];
    centpt.r = 255;
    centpt.g = 255;
    centpt.b = 255;
    obj_cloud->points.push_back(centpt);
    centpt.x += 0.005;
    obj_cloud->points.push_back(centpt);
    centpt.x -= 0.01;
    obj_cloud->points.push_back(centpt);
    centpt.x += 0.005;
    centpt.y += 0.005;
    obj_cloud->points.push_back(centpt);
    centpt.y -= 0.01;
    obj_cloud->points.push_back(centpt);
    centpt.y += 0.005;
    centpt.z += 0.005;
    obj_cloud->points.push_back(centpt);
    centpt.z -= 0.01;
    obj_cloud->points.push_back(centpt);
}


void PCSSegmentation::segmentation(PCS* input, PCS* output, int maxTime, int downsample)
{
    objects.clear();
    struct timespec t1, t2;
    double elapsed_time;
    volatile long long i;
    clock_gettime(CLOCK_MONOTONIC,  &t1);

    cout<<"nT: "<<input->nTime<<" max Time: " <<maxTime<<endl;
    //ofstream myfile;
    //myfile.open ("/home/mustafasezer/Desktop/coord.txt");

    for(int t=0;t<maxTime;t++){
        cout << "t = " << t << "   ******************************"<< endl;
        CloudPtr pc_input = input->pcs.at(t);
        CloudPtr pc_output;
        pc_output.reset(new Cloud);

        Cloud incloud;

        incloud.height = 1;
        incloud.width =1;
        incloud.is_dense = false;
        incloud.points.resize (pc_input->width * pc_input->height);

        for(int i=0; i<pc_input->points.size(); i++){
            incloud.points.push_back(pc_input->points.at(i));
        }
        /*pcl::PointXYZRGB pt;
        pt.x = 0;
        pt.y = 0;
        pt.z = 0.1;
        pt.r = 0;
        pt.g = 1;
        pt.b = 0;
        cloud->points.push_back(pt);*/



        /*CloudPtr cloud = boost::shared_ptr< Cloud >(new Cloud());
        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
        pcl::fromROSMsg(*cloud,tempCloud);*/

        /*pcl::PointCloud<pcl::PointXYZRGB> pc_inp;
        pcl::fromROSMsg(pc_input,pc_inp);*/

        // euclidean clustering
        double tolerance = 0.04;        //def: 0.02
        double minSize = 25;            //def: 5
        double maxSize = 10000;
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (pc_input);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (tolerance);
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud(pc_input);
        ec.extract (cluster_indices);


        if(objects.size() == 0){
            //Create an object for each blob at the beginning
            int id = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                /*//if(t==12){
                Eigen::Vector4f xyz_centroid;
                //pcl::compute3DCentroid (incloud, *it, xyz_centroid);
                EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
                //computeCovarianceMatrixNormalized (incloud, xyz_centroid, covariance_matrix);
                computeMeanAndCovarianceMatrix(incloud, *it, covariance_matrix, xyz_centroid);
                EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
                EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
                pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
                Eigen::Vector3f axes = K * Eigen::Vector3f(sqrt(eigen_values(0)), sqrt(eigen_values(1)), sqrt(eigen_values(2)));
                Eigen::Vector3f center = Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

                //New object
                Object obj(objects.size(), center, covariance_matrix, eigen_values, eigen_vectors, axes);
                if(obj.trainAppearance(pc_input, *it)){
                    objects.push_back(obj);
                    obj.blobID = id;
                    std::vector<int> dummy;
                    dummy.push_back(id);
                    associations.push_back(dummy);

                    cout << "New Object " << obj.id << " at t:" << t << " Center:";
                    for(int ii=0; ii<3; ii++){
                        std::cout << " ";
                        std::cout << obj.e.center[ii];
                    }
                    std::cout << std::endl;
                }*/

                Object obj(objects.size(), &incloud, *it);
                if(obj.GMM.isTrained()){
                    objects.push_back(obj);
                    obj.blobID = id;

                    std::vector<int> dummy;
                    associations.push_back(dummy);
                    associations[id].push_back(obj.id);

                    cout << "New Object " << obj.id << " of blob " << id << " at t:" << t << " Center:";
                    for(int ii=0; ii<3; ii++){
                        std::cout << " ";
                        std::cout << obj.e.center[ii];
                    }
                    std::cout << std::endl;
                }
                else{
                    std::cout << "ERROR: GMM not trained\n";
                }

                objects[id].pointIndices.clear();
                objects[id].pointIndices = it->indices;
                id++;
                //}
                visualizeObjectCenter(pc_output, obj.e.center);
            }
            for(int j=0; j<objects.size(); j++){
                showObject(j, pc_input, pc_output);
            }
        }
        else{
            //TODO: Be sure about the place of associations reset!!!
            //Reset associations
            for(int i=0; i<associations.size(); i++){
                associations[i].clear();
            }
            associations.clear();

            //For all objects
            for (int obj_ind=0; obj_ind<objects.size(); obj_ind++){
                if(objects[obj_ind].false_positive == true){
                    continue;
                }
                objects[obj_ind].pointIndices.clear();
                objects[obj_ind].blobs.clear();
                if(objects[obj_ind].disappeared == true){
                    //occluder'ının ellipsoid'ini inherit etmeli
                    //continue;
                }
                else if(objects[obj_ind].occluded == true){
                    /*CloudPtr cldptr = objects[obj_ind].cloud;
                    cldptr->points.size();
                    for(int ii=0; ii<cldptr->points.size(); ii++){
                        PointT point = cldptr->points[ii];
                        point.r = r[objects[obj_ind].id];
                        point.g = g[objects[obj_ind].id];
                        point.b = b[objects[obj_ind].id];
                        pc_output->points.push_back(point);
                    }*/
                    //continue;
                }
                double max_compatibility = 0;
                double max_compatibility_emax = 0;
                //For all blobs
                int id = 0;
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    if(associations.size() == id){
                        std::vector<int> dummy;
                        associations.push_back(dummy);
                    }
                    //Find points of intersection and calculate compatibility
                    double sum_compatibility = 0;
                    double sum_compatibility_emax = 0;
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                        PointT point = pc_input->points[*pit];

                        /*if(objects[obj_ind].distance(point) <= 1 || objects[obj_ind].distance(point, objects[obj_ind].e_max) <= 1){
                            sum_compatibility += objects[obj_ind].pixelCompatibility(Object::rgb2UV(point));
                        }*/

                        if(objects[obj_ind].distance(point) <= 1){
                            sum_compatibility += objects[obj_ind].pixelCompatibility(Object::rgb2UV(point));
                        }
                        else if(objects[obj_ind].distance(point, objects[obj_ind].e_max) <= 1){
                            sum_compatibility_emax += objects[obj_ind].pixelCompatibility(Object::rgb2UV(point));
                        }



                        /*for(int j=0; j<objects[obj_ind].ellipsoid_history_size; j++){
                            if(objects[obj_ind].distance(point) <= 1){
                                sum_compatibility_history[j] +=  objects[obj_ind].pixelCompatibility(Object::rgb2UV(point));
                            }
                        }*/
                    }
                    if(sum_compatibility > max_compatibility){
                        max_compatibility = sum_compatibility;
                        objects[obj_ind].blobID = id;
                    }
                    //sum_compatibility_emax>0 for more than 2 fractions, sum_compatibility_emax>max_compatibility_emax can only find two fractions of object
                    //and this is due to vector push back. Otherwise it would only take the best matching blob as a part of object
                    else if(sum_compatibility_emax > 0){
                        max_compatibility_emax = sum_compatibility_emax;
                        objects[obj_ind].blobs.push_back(id);
                    }
                    /*for(int j=0; j<objects[obj_ind].ellipsoid_history_size; j++){
                        if(sum_compatibility > max_compatibility){
                            max_compatibility = sum_compatibility;
                            objects[obj_ind].blobID = id;
                        }
                    }*/
                    id++;
                }
                //If object is associated with no blob --> object disappeared, Case 1b
                //TODO max compatibility tek bir pixel dahi olsa sıfır olmaz, o yüzden yukarıdakı for loop'a counter
                //eklenip eğer örn. 10 pixelden azsa disappeared falan denebilir.
                if(max_compatibility==0){
                    objects[obj_ind].disappeared = true;
                }
                else{
                    associations[objects[obj_ind].blobID].push_back(obj_ind);

                    for(int j=0; j<objects[obj_ind].blobs.size(); j++){
                        associations[objects[obj_ind].blobs[j]].push_back(obj_ind);
                    }

                }

                /*for(int j=0; j<objects[obj_ind].ellipsoid_history_size; j++){
                    if(max_compatibility_history[j]==0){
                        //objects[obj_ind].disappeared = true;
                    }
                    else{
                        associations[objects[obj_ind].blobID].push_back(obj_ind);
                    }
                }*/

            }

            //TODO for all blobs
            int id = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                if(associations[id].size() == 0){
                    //New object

                    //if(t==12){
                    /*Eigen::Vector4f xyz_centroid;
                    //pcl::compute3DCentroid (incloud, *it, xyz_centroid);
                    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
                    //computeCovarianceMatrixNormalized (incloud, xyz_centroid, covariance_matrix);
                    computeMeanAndCovarianceMatrix(incloud, *it, covariance_matrix, xyz_centroid);
                    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
                    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
                    pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
                    Eigen::Vector3f axes = K * Eigen::Vector3f(sqrt(eigen_values(0)), sqrt(eigen_values(1)), sqrt(eigen_values(2)));
                    Eigen::Vector3f center = Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

                    //New object
                    Object obj(objects.size(), center, covariance_matrix, eigen_values, eigen_vectors, axes);
                    if(obj.trainAppearance(pc_input, *it)){*/
                    Object obj(objects.size(), &incloud, *it);
                    if(obj.GMM.isTrained()){
                        objects.push_back(obj);
                        obj.blobID = id;
                        associations[id].push_back(obj.id);

                        cout << "New Object " << obj.id << " of blob " << id << " at t:" << t << " Center:";
                        for(int ii=0; ii<3; ii++){
                            std::cout << " ";
                            std::cout << obj.e.center[ii];
                        }
                        std::cout << std::endl;
                    }
                    else{
                        std::cout << "ERROR: GMM not trained\n";
                    }

                    //objects[obj.id].pointIndices.clear();
                    objects[obj.id].pointIndices = it->indices;

                    //}
                    //visualizeObjectCenter(pc_output, obj.e.center);
                }
                else if(associations[id].size() == 1 && objects[associations[id][0]].blobs.size()==0){
                    //One to one

                    std::cout << "Blob " << id << " 1-to-1 with object " << associations[id][0] << std::endl;

                    /*Eigen::Vector4f xyz_centroid;
                    //pcl::compute3DCentroid (incloud, *it, xyz_centroid);
                    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
                    //computeCovarianceMatrixNormalized (incloud, xyz_centroid, covariance_matrix);
                    computeMeanAndCovarianceMatrix(incloud, *it, covariance_matrix, xyz_centroid);
                    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
                    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
                    pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
                    Eigen::Vector3f axes = K * Eigen::Vector3f(sqrt(eigen_values(0)), sqrt(eigen_values(1)), sqrt(eigen_values(2)));
                    Eigen::Vector3f center = Vector3f(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

                    //Update object
                    objects[associations[id][0]].updateObject(associations[id][0], center, covariance_matrix, eigen_values, eigen_vectors, axes);*/
                    int objID = associations[id][0];
                    objects[objID].pointIndices = it->indices;
                    objects[objID].is1to1 = true;
                    objects[objID].divided = false;
                    //if(objects[objID].occluded == false){
                    if(objects[objID].e.volume >= objects[objID].e_max.volume){
                        if(objects[objID].updateAppearance(&incloud, *it)){
                            objects[objID].blobID = id;

                            //TODO The following push back is removed CHECK CHECK CHECK!!!
                            //associations[id].push_back(objects[associations[id][0]].id);

                            /*cout << "Object updated at t:" << t << " Center:";
                        for(int ii=0; ii<3; ii++){
                            std::cout << " ";
                            std::cout << objects[associations[id][0]].e.center[ii];
                        }
                        std::cout << std::endl;*/
                        }
                        else{
                            std::cout << "ERROR: Object appearance could not be updated\n";
                        }
                        //std::cout << "Blob " << id << " in one-to-one correspondence with object " << associations[id][0] << " at t=" << t << std::endl;
                    }
                    //Occluder list update
                    /*else{
                        if(objects.size()==2){
                            objects[objID].occluders.push_back(1-objID);
                        }
                        else{
                            for(int j=0; j<objects.size(); j++){

                            }
                        }
                    }*/
                    //visualizeObjectCenter(pc_output, objects[associations[id][0]].e.center);
                }
                //NEW CASE WITH AN OBJECT WITH MULTIPLE BLOBS
                else if(associations[id].size() == 1 && objects[associations[id][0]].blobs.size()>0){
                    //One to one NEW CASE WITH AN OBJECT WITH MULTIPLE BLOBS

                    std::cout << "Blob " << id << " is part of object " << associations[id][0] << std::endl;

                    int objID = associations[id][0];
                    objects[objID].is1to1 = true;
                    objects[objID].divided = true;
                    //Concatenate the blob's indices to the object
                    std::vector<int> AB;
                    AB.reserve( objects[objID].pointIndices.size() + it->indices.size() ); // preallocate memory
                    AB.insert(AB.end(), objects[objID].pointIndices.begin(), objects[objID].pointIndices.end());
                    AB.insert(AB.end(), it->indices.begin(), it->indices.end());
                    objects[objID].pointIndices.clear();
                    objects[objID].pointIndices = AB;
                    //TODO: DONT UPDATE UNTIL ALL INDICES ARE CONCATENATED

                    //Update object
                    //                    int objID = associations[id][0];
                    //                    if(objects[objID].occluded == false){
                    //                        if(objects[objID].updateAppearance(&incloud, *it)){
                    //                            objects[objID].blobID = id;

                    //TODO The following push back is removed CHECK CHECK CHECK!!!
                    //associations[id].push_back(objects[associations[id][0]].id);

                    /*cout << "Object updated at t:" << t << " Center:";
                        for(int ii=0; ii<3; ii++){
                            std::cout << " ";
                            std::cout << objects[associations[id][0]].e.center[ii];
                        }
                        std::cout << std::endl;*/
                    //                        }
                    //                        else{
                    //                            std::cout << "ERROR: Object appearance could not be updated\n";
                    //                        }
                    //std::cout << "Blob " << id << " in one-to-one correspondence with object " << associations[id][0] << " at t=" << t << std::endl;

                    //objects[objID].pointIndices.clear();
                    //                        objects[objID].pointIndices = it->indices;
                    //                    }
                    //Occluder list update
                    /*else{
                        if(objects.size()==2){
                            objects[objID].occluders.push_back(1-objID);
                        }
                        else{
                            for(int j=0; j<objects.size(); j++){

                            }
                        }
                    }*/
                    //visualizeObjectCenter(pc_output, objects[associations[id][0]].e.center);
                }
                else{
                    //Multiple Objects Case
                    std::cout << "Blob " << id << " has multiple objects: ";
                    for(int ii=0; ii<associations[id].size(); ii++)
                        std::cout << associations[id][ii] << " ";
                    std::cout << "at t=" << t;

                    double maxBhattCoeff = 0;
                    for(int j=0; j<associations[id].size()-1; j++){
                        for(int k=j+1; k<associations[id].size(); k++){
                            double bhattCoeff = Object::bhattacharyyaCoeff(objects[associations[id][j]].GMM, objects[associations[id][k]].GMM, 2);
                            if(bhattCoeff>maxBhattCoeff){
                                maxBhattCoeff = bhattCoeff;
                            }
                        }
                    }
                    cout << " Max Bhat Coeff = " << maxBhattCoeff << std::endl;

                    for(int j=0; j<associations[id].size(); j++){
                        objects[associations[id][j]].is1to1 = false;
                        objects[associations[id][j]].divided = false;
                    }

                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                        PointT point = pc_input->points[*pit];

                        if(maxBhattCoeff<0.95){
                            double maxRelation = 0;
                            int objID;
                            for(int j=0; j<associations[id].size(); j++){
                                double relation = objects[associations[id][j]].pixelRelation(point);
                                if(relation > maxRelation){
                                    maxRelation = relation;
                                    objID = associations[id][j];
                                }
                            }
                            objects[objID].pointIndices.push_back(*pit);
                        }
                        else{
                            double minDistance = 100000000;
                            int objID;
                            for(int j=0; j<associations[id].size(); j++){
                                double distance = objects[associations[id][j]].distance(point);
                                if(distance < minDistance){
                                    minDistance = distance;
                                    objID = associations[id][j];
                                }
                            }
                            objects[objID].pointIndices.push_back(*pit);
                        }
                    }
                }

                for(int ii=0; ii<associations[id].size(); ii++){
                    visualizeObjectCenter(pc_output, objects[associations[id][ii]].e.center);
                }
                //Increment blob id
                id++;
            }


            //False positive check
            for(int j=0; j<objects.size()-1; j++){
                for(int k=j+1; k<objects.size(); k++){
                    int l;
                    int counter = 0;
                    for(l=0; l<objects[k].pointIndices.size(); l++){
                        PointT point = pc_input->points[l];
                        if(objects[j].distance(point, objects[j].e_max)>1){
                            break;
                        }
                        //if(objects[j].distance(point, objects[j].e_max)<=1){
                        //    counter++;
                        //}
                    }
                    if(l==objects[k].pointIndices.size() && l>0){
                    //if(counter/100 >=objects[k].pointIndices.size()*50 && l>0){
                        std::cout << "Object " << k << " is totally in object " << j << std::endl;
                        std::cout << "Size: " << objects[j].pointIndices.size() << " --> ";
                        objects[k].false_positive = true;
                        std::vector<int> AB;
                        AB.reserve( objects[j].pointIndices.size() + objects[k].pointIndices.size() ); // preallocate memory
                        AB.insert(AB.end(), objects[j].pointIndices.begin(), objects[j].pointIndices.end());
                        AB.insert(AB.end(), objects[k].pointIndices.begin(), objects[k].pointIndices.end());
                        objects[k].pointIndices.clear();
                        //objects[k].e.axes =  Eigen::Vector3f(0.0000000001, 0.0000000001, 0.0000000001);
                        objects[j].pointIndices.clear();
                        objects[j].pointIndices = AB;
                        std::cout << objects[j].pointIndices.size() << std::endl;
                    }
                }
            }


            for(int j=0; j<objects.size(); j++){
                if(objects[j].false_positive){
                    continue;
                }
                showObject(j, pc_input, pc_output);
            }

            for(int j=0; j<objects.size(); j++){
                if(objects[j].pointIndices.size()==0)
                    continue;
                pcl::PointIndices ptInd;
                ptInd.indices = objects[j].pointIndices;
                if(objects[j].is1to1 || objects[j].divided){
                    objects[j].updatePosition(&incloud, ptInd);
                }
                else{
                    objects[j].updateEllipsePosition(&incloud, ptInd);
                }
            }
        }

        output->pcs.push_back(pc_output);
        t += downsample-1;
    }


    /*for(int k=0; k<objects.size(); k++){
        for(int l=0; l<objects.size(); l++){
            cout << "Bhatt " << k << " and " << l << " = " << Object::bhattacharyyaCoeff(objects[k].GMM, objects[l].GMM, 1) << endl;
        }
    }*/


    //myfile.close();
    clock_gettime(CLOCK_MONOTONIC,  &t2);
    elapsed_time = (t2.tv_sec - t1.tv_sec) + (double) (t2.tv_nsec - t1.tv_nsec) * 1e-9;
    std::cout << "Elapsed time: " << elapsed_time << std::endl;
}

void PCSSegmentation::segmentationRegionGrow(PCS* input, PCS* output){
    for(int t=0;t<input->nTime;t++){
        CloudPtr pc_input = input->pcs.at(t);
        CloudPtr pc_output;
        pc_output.reset(new Cloud);


        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (pc_input);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        /*pcl::IndicesPtr indices (new std::vector <int>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*indices);*/

        pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (pc_input);
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
        std::cout << "These are the indices of the points of the initial" <<
                     std::endl << "cloud that belong to the first cluster:" << std::endl;
        int counter = 0;
        while (counter < clusters[0].indices.size ())
        {
            std::cout << clusters[0].indices[counter] << ", ";
            counter++;
            if (counter % 10 == 0)
                std::cout << std::endl;
        }
        std::cout << std::endl;

        pc_output = reg.getColoredCloud ();

        output->pcs.push_back(pc_output);
    }
}


void PCSSegmentation::segmentationEuclidian(PCS* input, PCS* output){
    cout<<"nT: "<<input->nTime<<endl;
    for(int t=0;t<input->nTime;t++){
        CloudPtr pc_input = input->pcs.at(t);
        CloudPtr pc_output;
        pc_output.reset(new Cloud);

        // euclidean clustering
        double tolerance = 0.02;
        double minSize = 5;
        double maxSize = 10000;
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (pc_input);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (tolerance); // 2cm
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud( pc_input);
        ec.extract (cluster_indices);

        int id = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                PointT point = pc_input->points[*pit];
                point.r = r[id];
                point.g = g[id];
                point.b = b[id];
                pc_output->points.push_back(point);
            }

            id++;
        }

        output->pcs.push_back(pc_output);
    }
}
