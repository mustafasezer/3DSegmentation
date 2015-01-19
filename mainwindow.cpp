#include "mainwindow.h"
#include "../src-build/ui_mainwindow.h"

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
    ui->pushButton_process1->setEnabled(1);
    ui->pushButton_process1_2->setEnabled(true);
    ui->pushButton_process1_3->setEnabled(true);
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    if(isLoaded_raw)    pclViewer_raw->showPCat(value);
    if(isLoaded_gt)    pclViewer_gt->showPCat(value);
    if(isLoaded_result)    pclViewer_result->showPCat(value);
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
    ui->horizontalSlider->setValue(time);
}

void MainWindow::on_pushButton_stop_clicked()
{
    timer->stop();
    time = 0;
    ui->horizontalSlider->setValue(time);    
}

void MainWindow::on_pushButton_process1_clicked()
{
    pcs_result = new PCS();
    PCSSegmentation seg;
    int maxTime = ui->convertUntil->text().toInt();
    if(maxTime>pcs_raw->nTime || maxTime<0){
        maxTime = pcs_raw->nTime;
    }
    ui->horizontalSlider->setMaximum(maxTime-1);
    seg.segmentation(pcs_raw, pcs_result, maxTime);
    pclViewer_result = new PCLViewer();
    pclViewer_result->setPCS(pcs_result);
    pclViewer_result->setWindowTitle("PCS processing result data");
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
    pclViewer_result2->setWindowTitle("PCS processing result data");
    pclViewer_result2->setGeometry(382+0,600,700,500);
    pclViewer_result2->show();
    isLoaded_result = 1;
}

void MainWindow::on_pushButton_process1_3_clicked()
{
    ui->pushButton_process1_3->setEnabled(false);
    pcs_result3 = new PCS();
    PCSSegmentation seg;
    seg.segmentationEuclidian(pcs_raw, pcs_result3);
    pclViewer_result3 = new PCLViewer();
    pclViewer_result3->setPCS(pcs_result3);
    pclViewer_result3->setWindowTitle("PCS processing result data");
    pclViewer_result3->setGeometry(382+0,600,700,500);
    pclViewer_result3->show();
    isLoaded_result = 1;
}
