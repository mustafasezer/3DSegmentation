#ifndef PCSSEGMENTATION_H
#define PCSSEGMENTATION_H

#include "pcs.h"


class PCSSegmentation
{
public:
    PCSSegmentation();
    void segmentation(PCS* input, PCS* output);
    void segmentationRegionGrow(PCS* input, PCS* output);

    void visualizeObjectCenter(CloudPtr obj_cloud, Eigen::Vector3f center);

    //bool isInterior(PointT p, ellipse e);
    //ellipse createEllipse(Cloud incloud, pcl::PointIndices ind);

private:
    int maxID;
    int *r, *g, *b;

    std::vector< std::vector<int> > associations;


    //std::vector<Object*> objects;
};

#endif // PCSSEGMENTATION_H
