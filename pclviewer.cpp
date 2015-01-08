#include "pclviewer.h"
#include "../src-build/ui_pclviewer.h"
#include <QFileDialog>
#include <QMessageBox>

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));    
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    pSliderValueChanged (2);
    viewer->resetCamera ();
    viewer->setCameraPosition(1,0.5,0.5,0,0,0,0,0,1);
    viewer->addCoordinateSystem(0.1);
    ui->qvtkWidget->update ();

    pcs = new PCS();
}

void PCLViewer::setPCS(PCS* p)
{
    pcs = p;
    viewer->addPointCloud (pcs->pcs.at(0), "cloud");
    ui->qvtkWidget->update ();
}

void PCLViewer::showPCat(int time)
{
    viewer->updatePointCloud(pcs->pcs.at(time), "cloud");
    ui->qvtkWidget->update ();
}

void PCLViewer::pSliderValueChanged (int value)
{
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}
