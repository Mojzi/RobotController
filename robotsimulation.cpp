#include "robotsimulation.h"

#define H 3     // Height

#define L1 4.75    // Distance between S1 -> S2
#define L2 4.75    // Distance between S2 -> S3
#define L3 5.75    // Distance between S3 -> end

#define TESTA 160
/* =========== ZAKRESY SERW ROBOTA W STOPNIACH ================== */
#define SV0 (TESTA)     // servo 1 (rotating base)
#define SV1 (TESTA)     // servo 2 (first joint)
#define SV2 (TESTA)     // servo 3 (second joint)
#define SV3 (TESTA)     // servo 4 (third joint)
#define SV4 (TESTA)     // servo 5 (grabber rotation)
#define SV5 (TESTA)     // servo 6 (opening of grabber)

#define DRAW_SCALE 10
RobotSimulation::RobotSimulation(QGraphicsScene *scene, Servo *pout)
{
    calculatePosition(0,0,0,0, pout);
}

void RobotSimulation::draw(QGraphicsScene *scene)
{

    QBrush redBrush(Qt::red);
    QPen dotPen(Qt::black);
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    blackPen.setWidth(2);
    dotPen.setStyle(Qt::DashLine);
    int gridSize = 50;
    int gridOffset = 5;
    for(int i = -gridOffset; i < gridOffset; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addLine((i==0?0:segments[i-1].end.x)*DRAW_SCALE, (i==0?0:segments[i-1].end.y)*DRAW_SCALE, (segments[i].end.x)*DRAW_SCALE, (segments[i].end.y)*DRAW_SCALE);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addRect((segments[i].end.x)*DRAW_SCALE-1, (segments[i].end.y)*DRAW_SCALE-1, 3, 3, redPen, redBrush);
    }


}
void RobotSimulation::draw_td(QGraphicsScene *scene)
{

    QBrush redBrush(Qt::red);
    QPen dotPen(Qt::black);
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    blackPen.setWidth(2);
    dotPen.setStyle(Qt::DashLine);

    int gridSize = 50;
    int gridOffset = 5;
    for(int i = -gridOffset; i < gridOffset; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }

    for(int i=0; i<SEGMENTS; i++) {
        scene->addLine((i==0?0:segments[i-1].end.x)*DRAW_SCALE, -(i==0?0:segments[i-1].end.z)*DRAW_SCALE, (segments[i].end.x)*DRAW_SCALE, -(segments[i].end.z)*DRAW_SCALE);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addRect((segments[i].end.x)*DRAW_SCALE-1, -(segments[i].end.z)*DRAW_SCALE-1, 3, 3, redPen, redBrush);
    }
}


void matrixMultiplty(double in1[4][4], double in2[4][4], double out[4][4])
{
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                out[i][j] += in1[i][k]*in2[k][j];
}

void RobotSimulation::calculatePosition(float X, float Y, float Z, float P, Servo *pout)
{
    double Xb, Zb, p1, p2, t1, t2, t3, Q,
           V1[3] = {0}, V2[3] = {0}, V3[3] = {0},
           R0[4][4] = {0}, R1[4][4] = {0}, R2[4][4] = {0}, R3[4][4] = {0},
           T0[4][4] = {0}, T1[4][4] = {0}, T2[4][4] = {0}, T3[4][4] = {0},
           A0[4][4] = {0}, A1[4][4] = {0}, A2[4][4] = {0}, A3[4][4] = {0},
           P0[4][4] = {0}, P1[4][4] = {0}, P2[4][4] = {0}, P3[4][4] = {0};

    // INVERSE KINEMATICS
    Xb = (X - L3 * qCos(P * M_PI / 180)) / (2 * L1);
    Zb = (Z - H - L3 * qSin(P * M_PI / 180)) / (2 * L1);

    Q = qSqrt((1 / (Xb * Xb + Zb * Zb)) - 1);

    p1 = qAtan2(Zb + Q * Xb, Xb - Q * Zb) * (180 / M_PI);
    p2 = qAtan2(Zb - Q * Xb, Xb + Q * Zb) * (180 / M_PI);
    t1 = p1-90; // back to degrees
    t2 = p2-t1; // back to degrees
    t3 = P-p2;  // back to degrees

    segments[0].angle = qAtan2(Y,X) * 180 / M_PI;
    segments[0].angle = segments[0].angle*M_PI/180; //back to radians
    segments[1].angle = ((t1 + 90) * 2 * M_PI) / 360;  // back to radians
    segments[2].angle = ((t2 - 90) * 2 * M_PI) / 360;  // back to radians
    segments[3].angle = (t3 * 2 * M_PI) / 360;  // back to radians

    // FORWARD KINEMATICS
    R0[0][0] = qCos(segments[0].angle);
    R0[0][2] = -qSin(segments[0].angle);
    R0[1][1] = 1;
    R0[2][0] = -qSin(segments[0].angle);
    R0[2][2] = qCos(segments[0].angle);
    R0[3][3] = 1;
    R1[0][0] = qCos(segments[1].angle);
    R1[0][1] = -qSin(segments[1].angle);
    R1[1][0] = qSin(segments[1].angle);
    R1[1][1] = qCos(segments[1].angle);
    R1[2][2] = 1;
    R1[3][3] = 1;

    R2[0][0] = qCos(segments[2].angle);
    R2[0][1] = -qSin(segments[2].angle);
    R2[1][0] = qSin(segments[2].angle);
    R2[1][1] = qCos(segments[2].angle);
    R2[2][2] = 1;
    R2[3][3] = 1;

    R3[0][0] = qCos(segments[3].angle);
    R3[0][1] = -qSin(segments[3].angle);
    R3[1][0] = qSin(segments[3].angle);
    R3[1][1] = qCos(segments[3].angle);
    R3[2][2] = 1;
    R3[3][3] = 1;

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

    V1[0] = P1[0][3];
    V1[1] = P1[1][3];
    V2[0] = P2[0][3];
    V2[1] = P2[1][3];
    V3[0] = P3[0][3];
    V3[1] = P3[1][3];

    segments[0].end.set(P1[0][3], P1[1][3], P1[2][3]);
    segments[1].end.set(P2[0][3], P2[1][3], P2[2][3]);
    segments[2].end.set(P3[0][3], P3[1][3], P3[2][3]);

    pout->S6 = 128 - (segments[0].angle*180/M_PI)*(255/SV0);// first
    pout->S5 = 128 - (segments[0].angle*180/M_PI)*(255/SV1); // second
    pout->S4 = 128 - (segments[0].angle*180/M_PI)*(255/SV2); // third
    pout->S3 = 128 + (segments[0].angle*180/M_PI)*(255/SV3); // fourth
    pout->S2 = 128;
    pout->S1 = 128;
}
