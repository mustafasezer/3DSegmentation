#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcs.h"

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0);
    ~PCLViewer ();

public:
    void setPCS(PCS* p);
    void showPCat(int time);

public slots:    
    void pSliderValueChanged (int value);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;

private slots:


private:
    Ui::PCLViewer *ui;

    PCS *pcs;
    bool isLoaded;

};

#endif // PCLVIEWER_H
