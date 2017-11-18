#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
#include <QFileDialog>
#include "comminterface.h"
#include "mthread.h"
#define MX 40
#define MY 20

#define TX 10
#define TY 16

#define MAXPOINTS   2000
#define MAXSTEP     1000



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    CommInterface *comm;
    Servo Pin, Pout;
    Servo Srv[MAXPOINTS];
    Servo Program[MAXSTEP];
    ServoR Pos, dx;
    int ProgramCounter;
    int StepCount;
    int gcnt;
    int scnt;
    int pcnt;
    int gflag;
    int Runf;

private slots:
    void on_horizontalSlider_1_valueChanged(int value);

    void on_horizontalSlider_2_valueChanged(int value);

    void on_horizontalSlider_3_valueChanged(int value);

    void on_horizontalSlider_4_valueChanged(int value);

    void on_horizontalSlider_5_valueChanged(int value);

    void on_horizontalSlider_6_valueChanged(int value);

    void ex_th_tick();


    void on_actionNew_triggered();

    void on_actionAdd_Step_triggered();

    void on_actionEnd_Step_triggered();

    void on_actionRun_Program_triggered();

    void on_actionSave_triggered();

    void on_actionOpen_triggered();


private:
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *);
    MThread *th;
};

#endif // MAINWINDOW_H
