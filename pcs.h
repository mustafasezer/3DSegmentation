#ifndef PCS_H
#define PCS_H

#include <QtGui>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class PCS
{
public:
    PCS();
    bool loadFromFile(string file);

private:
    vector<string> split(string source, char delim, int rep=0);
    void push(double t, PointT point);
public:
    vector<CloudPtr> pcs;
    CloudPtr pc;
    int nTime;

private:
    int i;
    int cnt;
};

#endif // PCS_H
