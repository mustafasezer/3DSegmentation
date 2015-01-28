#include "mainwindow.h"
#include "../src-build/ui_mainwindow.h"



#include <QFileDialog>
#include <QMessageBox>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

//pviz = pcl::visualization::PCLVisualizer("test_viz", false);

pcl::visualization::PCLVisualizer pviz("visualize_mst", false);
int v1(0);
int v2(0);



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    time = 0;
    ms = 30;
    setGeometry(0,0,382,382);
    isLoaded_gt = 0;
    isLoaded_raw = 0;
    isLoaded_result = 0;
    isLoaded_result2 = 0;
    isLoaded_result3 = 0;
    showResultOnViewport2 = true;
    isLoaded_proc = false;

    vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
    ui->widget->SetRenderWindow (renderWindow);

    pviz.setupInteractor (ui->widget->GetInteractor (), ui->widget->GetRenderWindow ());
    pviz.getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);



    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pviz.initCameraParameters ();

    //int v1(0);
    pviz.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    //pviz.setBackgroundColor (0, 0, 0, v1);
    pviz.addText("Euclidian Clustering", 10, 10, 19, 1.0, 1.0, 1.0, "v1 text", v1);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //pviz.addPointCloud<pcl::PointXYZRGB> (cloud_xyz, "euclidian cloud", v1);

    //int v2(0);
    pviz.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    //pviz.setBackgroundColor (0.3, 0.3, 0.3, v2);
    pviz.addText("Mustafa Clustering", 10, 10, 19, 1.0, 1.0, 1.0, "v2 text", v2);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    //pviz.addPointCloud<pcl::PointXYZRGB> (cloud_xyz, "mustafa cloud", v2);

    pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "euclidian cloud");
    pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mustafa cloud");
    //pviz.addCoordinateSystem (1.0);

    //pviz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
    //pviz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

    //pviz.addPointCloud<pcl::PointXYZ>(cloud_xyz);
    //pviz.setBackgroundColor(0, 0, 0.1);


    pviz.resetCamera ();
    pviz.setCameraPosition(0.5,0.5,0.5,0,0,0,0,0,1,v1);
    pviz.setCameraPosition(1,0.5,0.5,0,0,0,0,0,1,v2);
    pviz.addCoordinateSystem(0.1);


    ui->widget->show();


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_PCS_triggered()
{
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open a PCL data file"),
                                                    currentDirectory);
    if (!filePath.isEmpty()){
        currentDirectory = QFileInfo(filePath).path();
        QString fileType = QFileInfo(filePath).suffix();

        if(fileType == QString("pcs")){

            QString filePath_raw = filePath;
            QString filePath_gt = filePath;

            caseName = filePath;
            caseName.truncate(filePath.lastIndexOf("_"));
            caseName.remove(0,caseName.lastIndexOf("/")+1);
            caseName.append("_proc.pcs");

            // loading raw data
            if(filePath.indexOf("_raw.pcs") != -1){
                filePath_gt.replace("_raw.pcs", "_gt.pcs");
            }
            else if(filePath.indexOf("_gt.pcs") != -1){
                filePath_raw.replace("_gt.pcs", "_raw.pcs");
            }

            pcs_raw = new PCS();
            if(!(pcs_raw->loadFromFile(filePath_raw.toStdString()))){
                QMessageBox msg;
                msg.setText("The selected file is not the proper type for loading a PCL data.");
                msg.exec();
            }
            else    isLoaded_raw = 1;

            pcs_gt = new PCS();
            if(!(pcs_gt->loadFromFile(filePath_gt.toStdString()))){
                QMessageBox msg;
                msg.setText("The selected file is not the proper type for loading a PCL data.");
                msg.exec();
            }
            else    isLoaded_gt = 1;
        }
    }
    if(isLoaded_raw){
        pclViewer_raw = new PCLViewer();
        pclViewer_raw->setPCS(pcs_raw);
        pclViewer_raw->setWindowTitle("PCS raw data");
        pclViewer_raw->setGeometry(382+0,0,700,500);
        pclViewer_raw->show();
    }
    if(isLoaded_gt){
        pclViewer_gt = new PCLViewer();
        pclViewer_gt->setPCS(pcs_gt);
        pclViewer_gt->setWindowTitle("PCS ground truth data");
        pclViewer_gt->setGeometry(382+700,0,700,500);
        pclViewer_gt->show();
    }
    setupgui();
}

void MainWindow::setupgui()
{
    ui->lineEdit_time->setText(QString::number(pcs_raw->nTime));
    ui->horizontalSlider->setMaximum(pcs_raw->nTime-1);
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setEnabled(1);
    ui->pushButton_start->setEnabled(1);
    ui->pushButton_stop->setEnabled(1);
    if(isLoaded_raw){
        ui->pushButton_process1->setEnabled(1);
        ui->pushButton_process1_2->setEnabled(true);
        ui->pushButton_process1_3->setEnabled(true);
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{

    if(isLoaded_result3){
        pviz.updatePointCloud(pcs_result3->pcs.at(value), "euclidian cloud");
    }
    if(isLoaded_proc && showResultOnViewport2==false){
        pviz.updatePointCloud(pcs_proc->pcs.at(value), "mustafa cloud");
    }
    else if(isLoaded_result && showResultOnViewport2==true){
        pviz.updatePointCloud(pcs_result->pcs.at(value), "mustafa cloud");
    }
    ui->widget->update ();

    if(isLoaded_raw)    pclViewer_raw->showPCat(value);
    if(isLoaded_gt)    pclViewer_gt->showPCat(value);
    if(isLoaded_proc)    pclViewer_proc->showPCat(value);
    if(isLoaded_result)    pclViewer_result->showPCat(value);
    if(isLoaded_result2)    pclViewer_result2->showPCat(value);
    if(isLoaded_result3)    pclViewer_result3->showPCat(value);
    time = value;
    ui->lineEdit_time->setText(QString::number(value));
}

void MainWindow::on_pushButton_start_clicked()
{
    if(ui->pushButton_start->isChecked()){
        timer->start(ms);
        ui->pushButton_start->setText("Pause");
    }
    else{
        timer->stop();
        ui->pushButton_start->setText("Start");
    }

}

void MainWindow::update()
{
    time ++;
    time += ui->downsample->text().toInt()-1;
    ui->horizontalSlider->setValue(time);
}

void MainWindow::on_pushButton_stop_clicked()
{
    timer->stop();
    time = 0;
    ui->horizontalSlider->setValue(time);
    ui->pushButton_start->setText("Start");
    ui->pushButton_start->setChecked(false);
}

void MainWindow::on_pushButton_process1_clicked()
{
    pcs_result = new PCS();
    PCSSegmentation seg;
    if(ui->convertUntil->text().size() == 0){
        ui->convertUntil->setText(QString::number(ui->horizontalSlider->maximum()));
    }
    int maxTime = ui->convertUntil->text().toInt();
    int downsample = ui->downsample->text().toInt();
    if(maxTime>pcs_raw->nTime || maxTime<0){
        maxTime = pcs_raw->nTime;
    }
    ui->horizontalSlider->setMaximum((maxTime)/downsample);
    seg.segmentation(pcs_raw, pcs_result, maxTime, downsample);
    if(ui->writePCSCheckBox->isChecked()){
        ofstream myfile;
        QString path = QDir::homePath().append("/Desktop/DHRI_Mustafa/Results/");
        if(QDir(path).exists() == false){
            QDir(path).mkpath(".");
        }
        myfile.open(path.append(caseName).toAscii());
        for(int t=0; t<pcs_result->pcs.size(); t++)
            for(int i=0; i<pcs_result->pcs[t]->points.size(); i++){
                PointT p = pcs_result->pcs[t]->points.at(i);
                myfile << t << " " << p.x << " " << p.y << " " << p.z << " " << (int)p.r << " " << (int)p.g << " " << (int)p.b << std::endl;
            }
        myfile.close();
    }
    ui->showResultRadioButton->setEnabled(true);
    if(showResultOnViewport2==true){
        pviz.removePointCloud("mustafa cloud", v2);
        CloudPtr cptr = pcs_result->pcs.at(0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cptr);
        pviz.addPointCloud<pcl::PointXYZRGB> (cptr, rgb, "mustafa cloud", v2);
    }


    pclViewer_result = new PCLViewer();
    pclViewer_result->setPCS(pcs_result);
    pclViewer_result->setWindowTitle("PCS Processing Result Data");
    pclViewer_result->setGeometry(382+0,600,700,500);
    pclViewer_result->show();
    isLoaded_result = 1;
    ui->pushButton_process1->setEnabled(false);
}

void MainWindow::on_pushButton_process1_2_clicked()
{
    ui->pushButton_process1_2->setEnabled(false);
    pcs_result2 = new PCS();
    PCSSegmentation seg;
    seg.segmentationRegionGrow(pcs_raw, pcs_result2);
    pclViewer_result2 = new PCLViewer();
    pclViewer_result2->setPCS(pcs_result2);
    pclViewer_result2->setWindowTitle("Region Growing Based Clustering");
    pclViewer_result2->setGeometry(382+0,600,700,500);
    pclViewer_result2->show();
    isLoaded_result2 = 1;
}

void MainWindow::on_pushButton_process1_3_clicked()
{
    ui->pushButton_process1_3->setEnabled(false);
    pcs_result3 = new PCS();
    PCSSegmentation seg;
    seg.segmentationEuclidian(pcs_raw, pcs_result3);
    CloudPtr cptr = pcs_result3->pcs.at(0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cptr);
    pviz.addPointCloud<pcl::PointXYZRGB> (cptr, rgb, "euclidian cloud", v1);


    pclViewer_result3 = new PCLViewer();
    pclViewer_result3->setPCS(pcs_result3);
    pclViewer_result3->setWindowTitle("Euclidian Clustering Result Data");
    pclViewer_result3->setGeometry(382+0,600,700,500);
    pclViewer_result3->show();
    isLoaded_result3 = 1;
}

void MainWindow::on_pushButton_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open a PCL data file"),
                                                    currentDirectory);
    if (!filePath.isEmpty()){
        currentDirectory = QFileInfo(filePath).path();
        QString fileType = QFileInfo(filePath).suffix();

        if(fileType == QString("pcs")){
            pcs_proc = new PCS();
            if(!(pcs_proc->loadFromFile(filePath.toStdString()))){
                QMessageBox msg;
                msg.setText("The selected file is not the proper type for loading a PCL data.");
                msg.exec();
            }
            else    isLoaded_proc = 1;

        }
        if(isLoaded_proc){
            pclViewer_proc = new PCLViewer();
            pclViewer_proc->setPCS(pcs_proc);
            pclViewer_proc->setWindowTitle("PCS Processing Result - Mustafa");
            pclViewer_proc->setGeometry(382+700,0,700,500);
            pclViewer_proc->show();
            ui->lineEdit_time->setText(QString::number(pcs_proc->nTime));
            ui->horizontalSlider->setMaximum(pcs_proc->nTime-1);
            ui->horizontalSlider->setMinimum(0);
            ui->horizontalSlider->setEnabled(1);
            ui->pushButton_start->setEnabled(1);
            ui->pushButton_stop->setEnabled(1);
            ui->showLoadedRadioButton->setEnabled(true);
            if(showResultOnViewport2==false){
                pviz.removePointCloud("mustafa cloud", v2);
                CloudPtr cptr = pcs_proc->pcs.at(0);
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cptr);
                pviz.addPointCloud<pcl::PointXYZRGB> (cptr, rgb, "mustafa cloud", v2);
            }

        }
    }
}

void MainWindow::on_showResultRadioButton_clicked()
{
    showResultOnViewport2 = true;
    pviz.removePointCloud("mustafa cloud", v2);
    CloudPtr cptr = pcs_result->pcs.at(time);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cptr);
    pviz.addPointCloud<pcl::PointXYZRGB> (cptr, rgb, "mustafa cloud", v2);
}

void MainWindow::on_showLoadedRadioButton_clicked()
{
    showResultOnViewport2 = false;
    pviz.removePointCloud("mustafa cloud", v2);
    CloudPtr cptr = pcs_proc->pcs.at(time);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cptr);
    pviz.addPointCloud<pcl::PointXYZRGB> (cptr, rgb, "mustafa cloud", v2);

}
