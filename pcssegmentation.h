#ifndef PCSSEGMENTATION_H
#define PCSSEGMENTATION_H

#include "pcs.h"

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



class PCSSegmentation
{
public:
    PCSSegmentation();
    void segmentation(PCS* input, PCS* output);
    void segmentationRegionGrow(PCS* input, PCS* output);

    //bool isInterior(PointT p, ellipse e);
    //ellipse createEllipse(Cloud incloud, pcl::PointIndices ind);

private:
    int maxID;
    int *r, *g, *b;
    //std::vector<Object*> objects;
};

#endif // PCSSEGMENTATION_H
