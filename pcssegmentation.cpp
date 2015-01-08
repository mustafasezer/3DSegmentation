#include "pcssegmentation.h"
#include <iostream>
#include <fstream>

/*#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>*/

#include <pcl/common/centroid.h>
#include <pcl/ros/conversions.h>

using namespace pcl;
using namespace Eigen;


float K = 2.795483482915108;        // 95% confidence

std::vector<Object> objects;


PCSSegmentation::PCSSegmentation()
{
    maxID = 100;

    srand(time(NULL));
    r = new int[maxID];
    g = new int[maxID];
    b = new int[maxID];

    for(int i=0;i<maxID;i++){
        r[i] = rand() % 256;
        g[i] = rand() % 256;
        b[i] = rand() % 256;
    }
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

void PCSSegmentation::segmentation(PCS* input, PCS* output)
{
    cout<<"nT: "<<input->nTime<<endl;
    ofstream myfile;
    myfile.open ("/home/mustafasezer/Desktop/coord.txt");
    for(int t=0;t<input->nTime;t++){
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
        ec.setInputCloud(pc_input);
        ec.extract (cluster_indices);


        int id = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            if(t==12){
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


                Object obj(objects.size(), center, covariance_matrix, eigen_values, eigen_vectors, axes);
                if(obj.trainAppearance(pc_input, *it)){
                    objects.push_back(obj);
                }

                //ellipse e = createEllipse(incloud, *it);

                //TODO
                //interiorPoints(e, *it);

                // Compute the distance threshold for sample selection
                //sample_dist_thresh_ = eigen_values.array ().sqrt ().sum () / 3.0;
                //sample_dist_thresh_ *= sample_dist_thresh_;
                // //return eigen_values.array ().sqrt ().sum () / 3.0;


                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
                    PointT point = pc_input->points[*pit];
                    myfile << t << " " << id << " " << point.x << " " << point.y << " " << point.z << std::endl;
                    point.r = r[id];
                    point.g = g[id];
                    point.b = b[id];
                    pc_output->points.push_back(point);
                }
                id++;
            }
        }
        output->pcs.push_back(pc_output);
    }
    myfile.close();
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
