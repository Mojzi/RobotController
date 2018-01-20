#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    comm =  new CommInterface();
    if(!comm->enabled)
    {
        QMessageBox mbox;
        std::string msg = "Podczas próby otwarcia portu wystąpił błąd:\n";
        msg.append(comm->error.c_str());
        mbox.setText(msg.c_str());
        mbox.setDefaultButton(QMessageBox::Ok);
        mbox.exec();
    }

    gflag=0;
    Runf=0;
    stepCount=100;
    programCounter=0;
    scnt=0;
    pcnt=0;
    gcnt=0;
    pos.S1=128;
    pos.S2=128;
    pos.S3=128;
    pos.S4=128;
    pos.S5=128;
    pos.S6=128;
    pout.S1=128;
    pout.S2=128;
    pout.S3=128;
    pout.S4=128;
    pout.S5=128;
    pout.S6=128;
    program[0]=pout;
    srv[0]=pout;

    th=new MThread();
    connect(th, SIGNAL(tick()), this, SLOT(ex_th_tick()));
    th->start(th->HighPriority);

    gflag=1;

    scene = new QGraphicsScene(this);
    scene->setSceneRect(-256, -256, 512, 512);

    scene2 = new QGraphicsScene(this);
    scene2->setSceneRect(-256, -256, 512, 512);
    ui->graphicsView_1->setScene(scene);
    ui->graphicsView_1->scale(1,-1); //flip to make going up be positive y;

    ui->graphicsView_2->setScene(scene2);
    ui->graphicsView_2->scale(1,-1); //flip to make going up be positive y;
    ui->label_error->setVisible(false);
    ui->label_error->setText("<font color='Red'>Position is out of reach!</font>");

    robotSimulation = new RobotSimulation(scene, &pout);
    calculateAndDrawRobot();

}

void MainWindow::paintEvent(QPaintEvent *)
{
    int sx=centralWidget()->x()+MX;
    int sy = centralWidget()->y()+MY;
    int ex = centralWidget()->width()+sx-2*MX;
    int ey = centralWidget()->height()+sy-2*MY;

    double dx = (ex-sx)/(double)TX;
    double dy = (ey-sy)/(double)TY;

    QPainter painter(this);
    QPen pen;

    pen.setWidth(1);
    painter.setRenderHint(QPainter::Antialiasing, true);

    pen.setColor(QColor(255,255,255,255));
    painter.setPen(pen);
    painter.setBrush(QColor(255,255,255,255));
    painter.drawRect(centralWidget()->x(), centralWidget()->y(),centralWidget()->width(), centralWidget()->height());
    pen.setStyle(Qt::DashLine);
    pen.setColor(QColor( 192,192,192,255 ));
    painter.setPen(pen);
    QFont font;
    font.setPointSize(8);
    painter.setFont(font);
    // ----- grid ------------
    for(int x=0; x<=TX; x++)
    painter.drawLine(QLineF(sx+x*dx, sy, sx+x*dx, ey));
    for(int y=0; y<=TY; y++)
    painter.drawLine(QLineF(sx, sy+y*dy, ex, sy+y*dy));
    // ------- description ----------
    pen.setColor(QColor( 0,0,0,255 ));

    painter.setPen(pen);
    for(int x=0; x<=TX; x++)
    painter.drawText(QPoint(sx-4+x*dx, ey+font.pointSize()),
    QString("%1").arg(x));
    for(int y=0; y<=TY; y++) {
        painter.drawText(QPoint(sx-20, sy+(font.pointSize()/2)+y*dy),
        QString("%1").arg(256-(256*y/TY)));
    }
    double lx=(ex-sx)/1000.0;
    double ly=(ey-sy)/256.0;
    if(gflag)
    {
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(2);
    pen.setColor(QColor( 255,0,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S1*ly, sx+i*lx, ey-srv[i].S1*ly));
    pen.setColor(QColor( 255,255,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S2*ly, sx+i*lx, ey-srv[i].S2*ly));
    pen.setColor(QColor( 0,255,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S3*ly, sx+i*lx, ey-srv[i].S3*ly));
    pen.setColor(QColor( 0,255,255,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S4*ly, sx+i*lx, ey-srv[i].S4*ly));
    pen.setColor(QColor( 255,128,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S5*ly, sx+i*lx, ey-srv[i].S5*ly));
    pen.setColor(QColor( 0,0,255,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-srv[i-1].S6*ly, sx+i*lx, ey-srv[i].S6*ly));
    }
}

MainWindow::~MainWindow()
{
    th->terminate();
    delete ui;
    delete comm;
}

void MainWindow::ex_th_tick()
{
    if(Runf==1)
    {
        ui->label_7->setText(QString("Step: %1").arg(pcnt+1));
        if(scnt==0)
        {
        pos.S1=(double)pout.S1;
        dx.S1= (program[pcnt].S1-pout.S1)/(double)stepCount;
        pos.S2=(double)pout.S2;
        dx.S2= (program[pcnt].S2-pout.S2)/(double)stepCount;
        pos.S3=(double)pout.S3;
        dx.S3= (program[pcnt].S3-pout.S3)/(double)stepCount;
        pos.S4=(double)pout.S4;
        dx.S4= (program[pcnt].S4-pout.S4)/(double)stepCount;
        pos.S5=(double)pout.S5;
        dx.S5= (program[pcnt].S5-pout.S5)/(double)stepCount;
        pos.S6=(double)pout.S6;
        dx.S6= (program[pcnt].S6-pout.S6)/(double)stepCount;
    }

    pos.S1+=dx.S1;
    pout.S1=(BYTE)pos.S1;
    pos.S2+=dx.S2;
    pout.S2=(BYTE)pos.S2;
    pos.S3+=dx.S3;
    pout.S3=(BYTE)pos.S3;
    pos.S4+=dx.S4;
    pout.S4=(BYTE)pos.S4;
    pos.S5+=dx.S5;
    pout.S5=(BYTE)pos.S5;
    pos.S6+=dx.S6;
    pout.S6=(BYTE)pos.S6;

    comm->send(pout);

    ui->horizontalSlider_1->setValue(pout.S1);
    ui->horizontalSlider_2->setValue(pout.S2);
    ui->horizontalSlider_3->setValue(pout.S3);
    ui->horizontalSlider_4->setValue(pout.S4);
    ui->horizontalSlider_5->setValue(pout.S5);
    ui->horizontalSlider_6->setValue(pout.S6);
    scnt++;
    if(scnt>=stepCount)
    {
        scnt=0;
        if(pcnt<programCounter)
        pcnt++;
    }
    if(pcnt>=programCounter)
    {
        Runf=0;
        ui->actionRun_Program->setEnabled(true);
    }
        srv[gcnt]=pout;
        gcnt++;
        ui->centralWidget->repaint();
        if(gcnt > stepCount*10)
        gcnt=0;
    }
}

void MainWindow::on_horizontalSlider_1_valueChanged(int value)
{
    ui->label->setText(QString().setNum(value));
    pout.S1=value;
    if(Runf==0)
    comm->send(pout);
}


void MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    ui->label_2->setText(QString().setNum(value));
    pout.S2=value;
    if(Runf==0)
    comm->send(pout);
}

void MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    ui->label_3->setText(QString().setNum(value));
    pout.S3=value;
    if(Runf==0)
    comm->send(pout);
}

void MainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    ui->label_4->setText(QString().setNum(value));
    pout.S4=value;
    if(Runf==0)
    comm->send(pout);
}

void MainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    ui->label_5->setText(QString().setNum(value));
    pout.S5=value;
    if(Runf==0)
    comm->send(pout);
}

void MainWindow::on_horizontalSlider_6_valueChanged(int value)
{
    ui->label_6->setText(QString().setNum(value));
    pout.S6=value;
    if(Runf==0)
    comm->send(pout);
}

void MainWindow::on_actionNew_triggered()
{
    ui->actionRun_Program->setEnabled(false);
    ui->textBrowser->clear();
    gcnt=0;
    programCounter=0;
    pcnt=0;
    ui->label_7->setText(QString("Step: %1").arg(programCounter+1));
    ui->horizontalSlider_1->setValue(128);
    ui->horizontalSlider_2->setValue(128);
    ui->horizontalSlider_3->setValue(128);
    ui->horizontalSlider_4->setValue(128);
    ui->horizontalSlider_5->setValue(128);
    ui->horizontalSlider_6->setValue(128);

    ui->dockWidget->setEnabled(true);

    ui->actionAdd_Step->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
    ui->actionRun_Program->setEnabled(false);

    pout.S1=128;
    pout.S2=128;
    pout.S3=128;
    pout.S4=128;
    pout.S5=128;
    pout.S6=128;

    program[0]=pout;
    srv[0]=pout;
}

void MainWindow::on_actionAdd_Step_triggered()
{
    program[programCounter]=pout;
    ui->label_7->setText(QString("Step: %1").arg(programCounter+1));
    ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
    programCounter+1, pout.S1, pout.S2, pout.S3, pout.S4, pout.S5, pout.S6));
    ui->actionEnd_Step->setEnabled(true);
    ui->actionRun_Program->setEnabled(false);
    if(programCounter<MAXSTEP)
        programCounter++;
}

void MainWindow::on_actionEnd_Step_triggered()
{
    ui->actionRun_Program->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
}

void MainWindow::on_actionRun_Program_triggered()
{
    scnt=0;
    pcnt=0;
    gcnt=0;
    Runf=1;
    ui->actionRun_Program->setEnabled(false);
}

void MainWindow::on_actionSave_triggered()
{
    QString FileName;
    FileName = QFileDialog::getSaveFileName( this , tr( "Open File" ), "/home/robot" , tr( "Robot Files (*.txt *.rob)" ));
    QFile file(FileName);
    if (!file. open ( QIODevice::WriteOnly | QIODevice :: Text ))
        return ;
    file.write(( char *) program , sizeof ( Servo )* programCounter );
    file.close();
}

void MainWindow::on_actionOpen_triggered()
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this, tr("Open FIle"), "/home/robot", tr("Robot Files (*.txt *.rob)"));
    QFile file(fileName);
    if (!file. open ( QIODevice::ReadOnly | QIODevice :: Text ))
        return ;
    int size = file.read((char *) program, sizeof(Servo)*MAXSTEP);
    file.close();
    if(size == -1) {
        return;
    }

    programCounter = size/sizeof(Servo);
    ui->label_7->clear();
    ui->textBrowser->clear();

    for(int i = 0; i < programCounter; i++)
    {
        pout = program[i];
        ui->label_7->setText(QString("Step: %1").arg(programCounter+1));
        ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
        i+1, pout.S1, pout.S2, pout.S3, pout.S4, pout.S5, pout.S6));
        ui->horizontalSlider_1->setValue(pout.S1);
        ui->horizontalSlider_2->setValue(pout.S2);
        ui->horizontalSlider_3->setValue(pout.S3);
        ui->horizontalSlider_4->setValue(pout.S4);
        ui->horizontalSlider_5->setValue(pout.S5);
        ui->horizontalSlider_6->setValue(pout.S6);
    }
    ui->actionAdd_Step->setEnabled(true);
    ui->actionRun_Program->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
}



void MainWindow::calculateAndDrawRobot()
{

    scene->clear();
    scene2->clear();

    ui->graphicsView_1->items().clear();
    float x = ui->spinBox_x->value();
    float y = ui->spinBox_y->value();
    float z = ui->spinBox_z->value();
    float p = ui->spinBox_p->value();
    float angles[4];
    if(robotSimulation->calculatePosition(x, y, z, p, &pout, angles))
    {
        ui->label_error->setVisible(false);
        robotSimulation->draw(scene);
        robotSimulation->draw_td(scene2);

        ui->label_angle_0->setText(QString::number(angles[0]));
        ui->label_angle_1->setText(QString::number(angles[1]));
        ui->label_angle_2->setText(QString::number(angles[2]));
        ui->label_angle_3->setText(QString::number(angles[3]));
    }
    else
    {
        ui->label_error->setVisible(true);
        robotSimulation->draw(scene);
        robotSimulation->draw_td(scene2);
    }

    ui->horizontalSlider_1->setValue(pout.S1);
    ui->horizontalSlider_2->setValue(pout.S2);
    ui->horizontalSlider_3->setValue(pout.S3);
    ui->horizontalSlider_4->setValue(pout.S4);
    ui->horizontalSlider_5->setValue(pout.S5);
    ui->horizontalSlider_6->setValue(pout.S6);

}

void MainWindow::on_spinBox_x_valueChanged(double arg1)
{
    calculateAndDrawRobot();
}

void MainWindow::on_spinBox_y_valueChanged(double arg1)
{

    calculateAndDrawRobot();
}

void MainWindow::on_spinBox_z_valueChanged(double arg1)
{

    calculateAndDrawRobot();
}

void MainWindow::on_spinBox_p_valueChanged(double arg1)
{
    calculateAndDrawRobot();
}
