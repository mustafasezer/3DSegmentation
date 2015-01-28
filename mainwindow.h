#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "pclviewer.h"
#include "pcs.h"
#include "pcssegmentation.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //pcl::visualization::PCLVisualizer pviz;

private slots:
    void on_actionOpen_PCS_triggered();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_start_clicked();

    void update();

    void on_pushButton_stop_clicked();

    void on_pushButton_process1_clicked();

    void on_pushButton_process1_2_clicked();

    void on_pushButton_process1_3_clicked();

    void on_pushButton_clicked();

    void on_showResultRadioButton_clicked();

    void on_showLoadedRadioButton_clicked();

private:
    void setupgui();

private:
    Ui::MainWindow *ui;
    PCLViewer *pclViewer_raw, *pclViewer_result, *pclViewer_result2, *pclViewer_result3, *pclViewer_gt, *pclViewer_proc;
    PCS *pcs_raw, *pcs_result, *pcs_result2, *pcs_result3, *pcs_gt, *pcs_proc;

    QString currentDirectory;
    QString caseName;
    bool isLoaded_raw, isLoaded_gt, isLoaded_proc, isLoaded_result, isLoaded_result2, isLoaded_result3;
    QTimer *timer;
    int time;
    int ms;
    bool showResultOnViewport2;
};

#endif // MAINWINDOW_H
