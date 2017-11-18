#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    comm =  new CommInterface();

    gflag=0;
    Runf=0;
    StepCount=100;
    ProgramCounter=0;
    scnt=0;
    pcnt=0;
    gcnt=0;
    Pos.S1=128;
    Pos.S2=128;
    Pos.S3=128;
    Pos.S4=128;
    Pos.S5=128;
    Pos.S6=128;
    Pout.S1=128;
    Pout.S2=128;
    Pout.S3=128;
    Pout.S4=128;
    Pout.S5=128;
    Pout.S6=128;
    Program[0]=Pout;
    Srv[0]=Pout;

    th=new MThread();
    connect(th, SIGNAL(tick()), this, SLOT(ex_th_tick()));
    th->start(th->HighPriority);

    gflag=1;
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
    // ----- siatka ------------
    for(int x=0; x<=TX; x++)
    painter.drawLine(QLineF(sx+x*dx, sy, sx+x*dx, ey));
    for(int y=0; y<=TY; y++)
    painter.drawLine(QLineF(sx, sy+y*dy, ex, sy+y*dy));
    // ------- opis ----------
    pen.setColor(QColor( 0,0,0,255 ));
    painter.setPen(pen);
    for(int x=0; x<=TX; x++)
    painter.drawText(QPoint(sx-4+x*dx, ey+font.pointSize()),
    QString("%1").arg(x));
    for(int y=0; y<=TY; y++)
    painter.drawText(QPoint(sx-20, sy+(font.pointSize()/2)+y*dy),
    QString("%1").arg(256-(256*y/TY)));
    double lx=(ex-sx)/1000.0;
    double ly=(ey-sy)/256.0;
    if(gflag)
    {
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(2);
    pen.setColor(QColor( 255,0,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S1*ly, sx+i*lx, ey-Srv[i].S1*ly));
    pen.setColor(QColor( 255,255,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S2*ly, sx+i*lx, ey-Srv[i].S2*ly));
    pen.setColor(QColor( 0,255,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S3*ly, sx+i*lx, ey-Srv[i].S3*ly));
    pen.setColor(QColor( 0,255,255,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S4*ly, sx+i*lx, ey-Srv[i].S4*ly));
    pen.setColor(QColor( 255,128,0,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S5*ly, sx+i*lx, ey-Srv[i].S5*ly));
    pen.setColor(QColor( 0,0,255,255 ));
    painter.setPen(pen);
    for ( int i=1; i<gcnt; i++ )
    painter.drawLine(QLineF(sx+(i-1)*lx, ey-Srv[i-1].S6*ly, sx+i*lx, ey-Srv[i].S6*ly));
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
        Pos.S1=(double)Pout.S1;
        dx.S1= (Program[pcnt].S1-Pout.S1)/(double)StepCount;
        Pos.S2=(double)Pout.S2;
        dx.S2= (Program[pcnt].S2-Pout.S2)/(double)StepCount;
        Pos.S3=(double)Pout.S3;
        dx.S3= (Program[pcnt].S3-Pout.S3)/(double)StepCount;
        Pos.S4=(double)Pout.S4;
        dx.S4= (Program[pcnt].S4-Pout.S4)/(double)StepCount;
        Pos.S5=(double)Pout.S5;
        dx.S5= (Program[pcnt].S5-Pout.S5)/(double)StepCount;
        Pos.S6=(double)Pout.S6;
        dx.S6= (Program[pcnt].S6-Pout.S6)/(double)StepCount;
    }

    Pos.S1+=dx.S1;
    Pout.S1=(BYTE)Pos.S1;
    Pos.S2+=dx.S2;
    Pout.S2=(BYTE)Pos.S2;
    Pos.S3+=dx.S3;
    Pout.S3=(BYTE)Pos.S3;
    Pos.S4+=dx.S4;
    Pout.S4=(BYTE)Pos.S4;
    Pos.S5+=dx.S5;
    Pout.S5=(BYTE)Pos.S5;
    Pos.S6+=dx.S6;
    Pout.S6=(BYTE)Pos.S6;

    comm->Send(Pout);

    ui->horizontalSlider_1->setValue(Pout.S1);
    ui->horizontalSlider_2->setValue(Pout.S2);
    ui->horizontalSlider_3->setValue(Pout.S3);
    ui->horizontalSlider_4->setValue(Pout.S4);
    ui->horizontalSlider_5->setValue(Pout.S5);
    ui->horizontalSlider_6->setValue(Pout.S6);
    scnt++;
    if(scnt>=StepCount)
    {
        scnt=0;
        if(pcnt<ProgramCounter)
        pcnt++;
    }
    if(pcnt>=ProgramCounter)
    {
        Runf=0;
        ui->actionRun_Program->setEnabled(true);
    }
        Srv[gcnt]=Pout;
        gcnt++;
        ui->centralWidget->repaint();
        if(gcnt > StepCount*10)
        gcnt=0;
    }
}

void MainWindow::on_horizontalSlider_1_valueChanged(int value)
{
    ui->label->setText(QString().setNum(value));
    Pout.S1=value;
    if(Runf==0)
    comm->Send(Pout);
}


void MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    ui->label_2->setText(QString().setNum(value));
    Pout.S2=value;
    if(Runf==0)
    comm->Send(Pout);
}

void MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    ui->label_3->setText(QString().setNum(value));
    Pout.S3=value;
    if(Runf==0)
    comm->Send(Pout);
}

void MainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    ui->label_4->setText(QString().setNum(value));
    Pout.S4=value;
    if(Runf==0)
    comm->Send(Pout);
}

void MainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    ui->label_5->setText(QString().setNum(value));
    Pout.S5=value;
    if(Runf==0)
    comm->Send(Pout);
}

void MainWindow::on_horizontalSlider_6_valueChanged(int value)
{
    ui->label_6->setText(QString().setNum(value));
    Pout.S6=value;
    if(Runf==0)
    comm->Send(Pout);
}

void MainWindow::on_actionNew_triggered()
{
    ui->actionRun_Program->setEnabled(false);
    ui->textBrowser->clear();
    gcnt=0;
    ProgramCounter=0;
    pcnt=0;
    ui->label_7->setText(QString("Step: %1").arg(ProgramCounter+1));
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

    Pout.S1=128;
    Pout.S2=128;
    Pout.S3=128;
    Pout.S4=128;
    Pout.S5=128;
    Pout.S6=128;

    Program[0]=Pout;
    Srv[0]=Pout;
}

void MainWindow::on_actionAdd_Step_triggered()
{
    Program[ProgramCounter]=Pout;
    ui->label_7->setText(QString("Step: %1").arg(ProgramCounter+1));
    ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
    ProgramCounter+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));
    ui->actionEnd_Step->setEnabled(true);
    ui->actionRun_Program->setEnabled(false);
    if(ProgramCounter<MAXSTEP)
        ProgramCounter++;
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
    file.write(( char *) Program , sizeof ( Servo )* ProgramCounter );
    file.close();
}

void MainWindow::on_actionOpen_triggered()
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this, tr("Open FIle"), "/home/robot", tr("Robot Files (*.txt *.rob)"));
    QFile file(fileName);
    if (!file. open ( QIODevice::ReadOnly | QIODevice :: Text ))
        return ;
    int size = file.read((char *) Program, sizeof(Servo)*MAXSTEP);
    file.close();
    if(size == -1) {
        return;
    }

    ProgramCounter = size/sizeof(Servo);
    ui->label_7->clear();
    ui->textBrowser->clear();

    for(int i = 0; i < ProgramCounter; i++)
    {
        Pout = Program[i];
        ui->label_7->setText(QString("Step: %1").arg(ProgramCounter+1));
        ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
        i+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));
        ui->horizontalSlider_1->setValue(Pout.S1);
        ui->horizontalSlider_2->setValue(Pout.S2);
        ui->horizontalSlider_3->setValue(Pout.S3);
        ui->horizontalSlider_4->setValue(Pout.S4);
        ui->horizontalSlider_5->setValue(Pout.S5);
        ui->horizontalSlider_6->setValue(Pout.S6);
    }
    ui->actionAdd_Step->setEnabled(true);
    ui->actionRun_Program->setEnabled(true);
    ui->actionEnd_Step->setEnabled(false);
}

