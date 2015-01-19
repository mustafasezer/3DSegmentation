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

private slots:
    void on_actionOpen_PCS_triggered();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_start_clicked();
    void update();
    void on_pushButton_stop_clicked();



    void on_pushButton_process1_clicked();

    void on_pushButton_process1_2_clicked();

    void on_pushButton_process1_3_clicked();

private:
    void setupgui();

private:
    Ui::MainWindow *ui;
    PCLViewer *pclViewer_raw, *pclViewer_result, *pclViewer_result2, *pclViewer_result3, *pclViewer_gt;
    PCS *pcs_raw, *pcs_result, *pcs_result2, *pcs_result3, *pcs_gt;

    QString currentDirectory;
    bool isLoaded_raw, isLoaded_gt, isLoaded_result;
    QTimer *timer;
    int time;
    int ms;
};

#endif // MAINWINDOW_H
