#include "robotsimulation.h"

#define H 3     // Wysokość ramienia
//#define L1 8    // Odległość między S1 -> S2
//#define L2 8    // Odległość między S2 -> S3
//#define L3 20    // Odległość między S3 -> końćem chwytaka

#define L1 4.75    // Odległość między S1 -> S2
#define L2 4.75    // Odległość między S2 -> S3
#define L3 5.75    // Odległość między S3 -> końćem chwytaka

#define TESTA 160
/* =========== ZAKRESY SERW ROBOTA W STOPNIACH ================== */
#define SV0 (TESTA)     // servo 1 (obrotowa podstawa)
#define SV1 (TESTA)     // servo 2 (pierwsze zgiecie)
#define SV2 (TESTA)     // ...
#define SV3 (TESTA)     // ...
#define SV4 (TESTA)     // servo 5 (obrot chwytaka)
#define SV5 (TESTA)     // servo 6 (rozchylenie chwytaka)

#define DRAW_SCALE 10
RobotSimulation::RobotSimulation(QGraphicsScene *scene, Servo *pout)
{

//    line = scene->addLine(5, 5, 100, 100, blackPen);
//    line = scene->addLine(100, 100, 200, 250, blackPen);
//    rectangle = scene->addRect(98, 98, 4, 4, redPen, redBrush);
    kinematics(0,0,0,0, pout);
    draw(scene);
}

void RobotSimulation::draw(QGraphicsScene *scene)
{
    QBrush redBrush(Qt::red);
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    QPen dotPen(Qt::lightGray);
    dotPen.setStyle(Qt::DashLine);

    int gridSize = 50;
    int gridOffset = 5;
    for(int i = -gridOffset; i < gridOffset; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }
    blackPen.setWidth(2);
    for(int i=0; i<SEGMENTS; i++) {
        arms[i] = scene->addLine((i==0?0:segments[i-1].end.x)*DRAW_SCALE, (i==0?0:segments[i-1].end.y)*DRAW_SCALE, (segments[i].end.x)*DRAW_SCALE, (segments[i].end.y)*DRAW_SCALE);
    }
    for(int i=0; i<SEGMENTS; i++) {
        joints[i] = scene->addRect((segments[i].end.x)*DRAW_SCALE-1, (segments[i].end.y)*DRAW_SCALE-1, 3, 3, redPen, redBrush);
    }


}
void RobotSimulation::draw_td(QGraphicsScene *scene, float x, float y, float z)
{

    QBrush redBrush(Qt::red);
    QPen blackPen(Qt::black);
    QPen dotPen(Qt::lightGray);

    float len, qq, yp, xp;

    len = qSqrt(qPow(x,2)+qPow(y,2));
    qq = qSqrt(qPow(L1,2)-qPow(z/2,2));

    yp = -(qq*(y/len));
    xp = -(qq*(x/len));

    int gridSize = 50;
    int gridOffset = 5;
    for(int i = -gridOffset; i < gridOffset; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }

    scene->addLine(x*DRAW_SCALE,y*DRAW_SCALE,xp*DRAW_SCALE,yp*DRAW_SCALE,blackPen);
}
void matrixMultiplty(double in1[4][4], double in2[4][4], double out[4][4])
{
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                out[i][j] += in1[i][k]*in2[k][j];
}

void RobotSimulation::kinematics(float x, float y, float z, float p, Servo *pout)
{
    char X,Y,Z,P;
    double Xb, Zb, p1, p2, t1, t2, t3, Q,
           Ph0, Ph1, Ph2, Ph3,
           Svo1, Svo2, Svo3, Svo4,Svo5, Svo6,
           V1[2] = {0}, V2[2] = {0}, V3[3] = {0},
           Gx[4] = {0}, Gy[4] = {0}, Gz[4] = {0},
           R0[4][4] = {0}, R1[4][4] = {0}, R2[4][4] = {0}, R3[4][4] = {0},
           T0[4][4] = {0}, T1[4][4] = {0}, T2[4][4] = {0}, T3[4][4] = {0},
           A0[4][4] = {0}, A1[4][4] = {0}, A2[4][4] = {0}, A3[4][4] = {0},
           P0[4][4] = {0}, P1[4][4] = {0}, P2[4][4] = {0}, P3[4][4] = {0};

    X = x;
    Y = y;
    Z = z;
    P = p; //ui->spbP->value();  // ?????????????
  //  ui->label_3->setText(QString().sprintf("X = %d",X));
  //  ui->label_4->setText(QString().sprintf("Y = %d",Y));
   // ui->label_5->setText(QString().sprintf("Z = %d",Z));
   // ui->label->setText(QString().sprintf("%f",qCos(qAtan2(ui->spbY->value(),X) * 180 / M_PI)));

    // -------------------- KINEMATYKA ODWROTNA --------------------------
    Xb = (X - L3 * qCos(P * M_PI / 180)) / (2 * L1);
    Zb = (Z - H - L3 * qSin(P * M_PI / 180)) / (2 * L1);
//    ui->label_3->setText(QString().sprintf("Xb = %f,Zb = %f",Xb,Zb));

    Q = qSqrt((1 / (Xb * Xb + Zb * Zb)) - 1);
//    ui->label_4->setText(QString().sprintf("q = %f",(1/(Xb * Xb + Zb * Zb))-1));

    p1 = qAtan2(Zb + Q * Xb, Xb - Q * Zb) * (180 / M_PI);
    p2 = qAtan2(Zb - Q * Xb, Xb + Q * Zb) * (180 / M_PI);
   // ui->label_6->setText(QString().sprintf("%f, %f, %f, %f",R0[3][0],R0[3][1],R0[3][2],R0[3][3]));
    t1 = p1-90; // stopnie
    t2 = p2-t1; // stopnie
    t3 = P-p2;  // stopnie

    Ph0 = qAtan2(Y,X) * 180 / M_PI;  // stopnie
    Ph0 = Ph0*M_PI/180;
    Ph1 = ((t1 + 90) * 2 * M_PI) / 360;  // radiany
    Ph2 = ((t2 - 90) * 2 * M_PI) / 360;  // radiany
    Ph3 = (t3 * 2 * M_PI) / 360;  // radiany
//    ui->label->setText(QString::number(Ph3));
//    ui->test_2->setText(QString::number(Ph1));
//    ui->test_3->setText(QString::number(Ph2));
//    ui->test_4->setText(QString::number(Ph3));

    // --------------------- KINEMATYKA PROSTA ---------------------------
    R0[0][0] = qCos(Ph0);
    R0[0][2] = -qSin(Ph0);
    R0[1][1] = 1;
    R0[2][0] = -qSin(Ph0);
    R0[2][2] = qCos(Ph0);
    R0[3][3] = 1;
//    ui->label_2->setText(QString().sprintf("%f",qCos(qAtan2(-20,X) * 180 / M_PI)));
    R1[0][0] = qCos(Ph1);
    R1[0][1] = -qSin(Ph1);
    R1[1][0] = qSin(Ph1);
    R1[1][1] = qCos(Ph1);
    R1[2][2] = 1;
    R1[3][3] = 1;

    R2[0][0] = qCos(Ph2);
    R2[0][1] = -qSin(Ph2);
    R2[1][0] = qSin(Ph2);
    R2[1][1] = qCos(Ph2);
    R2[2][2] = 1;
    R2[3][3] = 1;

    R3[0][0] = qCos(Ph3);
    R3[0][1] = -qSin(Ph3);
    R3[1][0] = qSin(Ph3);
    R3[1][1] = qCos(Ph3);
    R3[2][2] = 1;
    R3[3][3] = 1;
   // ui->label_3->setText(QString().sprintf("%f, %f, %f, %f",R0[0][0],R0[0][1],R0[0][2],R0[0][3]));
   // ui->label_4->setText(QString().sprintf("%f, %f, %f, %f",R0[1][0],R0[1][1],R0[1][2],R0[1][3]));
   // ui->label_5->setText(QString().sprintf("%f, %f, %f, %f",R0[2][0],R0[2][1],R0[2][2],R0[2][3]));
   // ui->label_6->setText(QString().sprintf("%f, %f, %f, %f",R0[3][0],R0[3][1],R0[3][2],R0[3][3]));


    T0[0][0] = 1;
    T0[1][1] = 1;
    T0[1][3] = H;
    T0[2][2] = 1;
    T0[3][3] = 1;

    T1[0][0] = 1;
    T1[1][1] = 1;
    T1[0][3] = L1;
    T1[2][2] = 1;
    T1[3][3] = 1;

    T2[0][0] = 1;
    T2[1][1] = 1;
    T2[0][3] = L2;
    T2[2][2] = 1;
    T2[3][3] = 1;

    T3[0][0] = 1;
    T3[1][1] = 1;
    T3[0][3] = L3;
    T3[2][2] = 1;
    T3[3][3] = 1;

    matrixMultiplty(R0,T0,A0);
    matrixMultiplty(R1,T1,A1);
    matrixMultiplty(R2,T2,A2);
    matrixMultiplty(R3,T3,A3);


    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            P0[i][j] = A0[i][j];

    matrixMultiplty(P0,A1, P1);
    matrixMultiplty(P1,A2, P2);
    matrixMultiplty(P2,A3, P3);

//    for (int i=0; i<4; i++)
//        for (int j=0; j<4; j++)
//            for (int k=0; k<4; k++)
//                P1[i][j] += A0[i][k]*A1[k][j];

//    for (int i=0; i<4; i++)
//        for (int j=0; j<4; j++)
//            for (int k=0; k<4; k++)
//                P2[i][j] += P1[i][k]*A2[k][j];
//    for (int i=0; i<4; i++)
//        for (int j=0; j<4; j++)
//            for (int k=0; k<4; k++)
//                P3[i][j] += P2[i][k]*A3[k][j];
    V1[0] = P1[0][3];
    V1[1] = P1[1][3];
    V2[0] = P2[0][3];
    V2[1] = P2[1][3];
    V3[0] = P3[0][3];
    V3[1] = P3[1][3];

    segments[0].end.set(P1[0][3], P1[1][3]);
    segments[1].end.set(P2[0][3], P2[1][3]);
    segments[2].end.set(P3[0][3], P3[1][3]);

    /* =========================================================== */
    /* ===         USTALANIE WARTOSCI SERWOMECHANIZMOW         === */
    /* =========================================================== */
        Svo6 = 128 - (Ph0*180/M_PI)*(255/SV0);//128 - qFloor((Ph0*256)/SV0); // Svo1 = 128 - ((Ph0/(SV0/2))*128);
        Svo5 = 128 - (Ph3*180/M_PI)*(255/SV1); // DRUGIE OD DOLU
        Svo4 = 128 - (Ph2*180/M_PI)*(255/SV2); // TRZECIE OD DOLU
        Svo3 = 128 + (Ph1*180/M_PI)*(255/SV3); // CZWARTE OD DOLU
        Svo2 = 128;
        Svo1 = 128;
      //  ui->label_2->setText(QString().sprintf("%f",Ph3));
        pout->S6 = (int)Svo6;
        pout->S5 = (int)Svo5;
        pout->S4 = (int)Svo4;
        pout->S3 = (int)Svo3;
        pout->S2 = 128;
        pout->S1 = 128;
//        ui->lblProgramStep->setText(QString("Step: %1").arg(ProgramCounter+1));
//        ui->textBrowser->insertPlainText(QString().sprintf("%4d) %03d %03d %03d %03d %03d %03d\n",
//        ProgramCounter+1, Pout.S1, Pout.S2, Pout.S3, Pout.S4, Pout.S5, Pout.S6));

    Gx[0] = 0;
    Gx[1] = V1[0];
    Gx[2] = V2[0];
    Gx[3] = V3[0];

    Gy[0] = H;
    Gy[1] = V1[1];
    Gy[2] = V2[1];
    Gy[3] = V3[1];

    Gz[0] = 0;
    Gz[1] = 0;
    Gz[2] = 0;
    Gz[3] = 0;
}

void RobotSimulation::setValuesFromServo(Servo *servo)
{
//        double Ph0, Ph1, Ph2, Ph3,
//                Svo6, Svo5, Svo4, Svo3;

//        Svo6 = pout->S6;
//        Svo5 = pout->S5;
//        Svo4 = pout->S4;
//        Svo3 = pout->S3;

//        Ph0 = Svo6/((180/M_PI)*(255/SV0));
//        Ph1 = Svo5/((180/M_PI)*(255/SV1));
//        Ph2 = Svo4/((180/M_PI)*(255/SV2));
//        Ph3 = Svo3/((180/M_PI)*(255/SV3));

//        y1 = H * sin(Ph0);
//        x1 = H * cos(Ph0);
//        y2 = L1 * sin(Ph0);
//        x2 = L1 * cos(Ph0);
//        y3 = H * sin(Ph0);
//        x3 = H * cos(Ph0);



//    Xb = (X - L3 * qCos(P * M_PI / 180)) / (2 * L1);
//    Zb = (Z - H - L3 * qSin(P * M_PI / 180)) / (2 * L1);
////    ui->label_3->setText(QString().sprintf("Xb = %f,Zb = %f",Xb,Zb));

//    Q = qSqrt((1 / (Xb * Xb + Zb * Zb)) - 1);
////    ui->label_4->setText(QString().sprintf("q = %f",(1/(Xb * Xb + Zb * Zb))-1));

//    p1 = qAtan2(Zb + Q * Xb, Xb - Q * Zb) * (180 / M_PI);
//    p2 = qAtan2(Zb - Q * Xb, Xb + Q * Zb) * (180 / M_PI);
//   // ui->label_6->setText(QString().sprintf("%f, %f, %f, %f",R0[3][0],R0[3][1],R0[3][2],R0[3][3]));
//    t1 = p1-90; // stopnie
//    t2 = p2-t1; // stopnie
//    t3 = P-p2;  // stopnie

//    Ph0 = qAtan2(Y,X) * 180 / M_PI;  // stopnie
//    Ph0 = Ph0*M_PI/180;
//    Ph1 = ((t1 + 90) * 2 * M_PI) / 360;  // radiany
//    Ph2 = ((t2 - 90) * 2 * M_PI) / 360;  // radiany
//    Ph3 = (t3 * 2 * M_PI) / 360;  // radiany
}
