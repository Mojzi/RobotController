#include "robotsimulation.h"

#define H 6.5     // Height

//#define L1 4.75    // Distance between S1 -> S2
//#define L2 4.75    // Distance between S2 -> S3
//#define L3 5.75    // Distance between S3 -> end

#define L3 17.2
#define L2 8.1
#define L1 8.0

#define TESTA 125
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
}

void RobotSimulation::draw(QGraphicsScene *scene)
{

    QBrush redBrush(Qt::red);
    QPen dotPen(Qt::black);
    QBrush lightGrayBrush(Qt::lightGray);
    QPen lightGrayPen(Qt::lightGray);
    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    blackPen.setWidth(2);
    dotPen.setStyle(Qt::DashLine);

    scene->addRect(-10.5*DRAW_SCALE, 0, 21*DRAW_SCALE, 5.8*DRAW_SCALE, lightGrayPen, lightGrayBrush);
    scene->addLine(0,0, 0, H*DRAW_SCALE);

    int gridSize = 100;
    int gridOffset = 5;
    for(int i = -gridSize; i < gridSize; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addLine((i==0?0:segments[i-1].end.x)*DRAW_SCALE, (i==0?H:segments[i-1].end.z)*DRAW_SCALE, (segments[i].end.x)*DRAW_SCALE, (segments[i].end.z)*DRAW_SCALE);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addRect((segments[i].end.x)*DRAW_SCALE-1, (segments[i].end.z)*DRAW_SCALE-1, 3, 3, redPen, redBrush);
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

    int gridSize = 100;
    int gridOffset = 5;
    for(int i = -gridSize; i < gridSize; i++) {
        scene->addLine(-gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, gridSize*DRAW_SCALE, i*DRAW_SCALE*gridOffset, dotPen);
        scene->addLine(i*gridOffset*DRAW_SCALE, -gridSize*gridOffset, i*gridOffset*DRAW_SCALE, gridSize*gridOffset, dotPen);
    }

    for(int i=0; i<SEGMENTS; i++) {
        scene->addLine((i==0?0:segments[i-1].end.x)*DRAW_SCALE, -(i==0?0:segments[i-1].end.y)*DRAW_SCALE, (segments[i].end.x)*DRAW_SCALE, -(segments[i].end.y)*DRAW_SCALE);
    }
    for(int i=0; i<SEGMENTS; i++) {
        scene->addRect((segments[i].end.x)*DRAW_SCALE-1, -(segments[i].end.y)*DRAW_SCALE-1, 3, 3, redPen, redBrush);
    }
}


void matrixMultiplty(double in1[4][4], double in2[4][4], double out[4][4])
{
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            for (int k=0; k<4; k++)
                out[i][j] += in1[i][k]*in2[k][j];
}

bool checkValidity(double in[4][4])
{
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            if(qIsNaN(in[i][j]))
                return false;
    return true;
}

inline double clamp(double x, double a, double b)
{
    return x < a ? a : (x > b ? b : x);
}

bool RobotSimulation::calculatePosition(float X, float Y, float Z, float P, Servo *pout, float angles[4])
{
    double Xb, Zb, p1, p2, t1, t2, t3, Q, angle[4],
           R0[4][4] = {0}, R1[4][4] = {0}, R2[4][4] = {0}, R3[4][4] = {0},
           T0[4][4] = {0}, T1[4][4] = {0}, T2[4][4] = {0}, T3[4][4] = {0},
           A0[4][4] = {0}, A1[4][4] = {0}, A2[4][4] = {0}, A3[4][4] = {0},
           P0[4][4] = {0}, P1[4][4] = {0}, P2[4][4] = {0}, P3[4][4] = {0};

    angle[0] =  qAtan2(Y,X);
    X = qSqrt(X*X + Y*Y);
    // INVERSE KINEMATICS
    Xb = (X - L3 * qCos(P * M_PI / 180)) / (2 * L1);
    Zb = (Z - H - L3 * qSin(P * M_PI / 180)) / (2 * L1);
    double Yb = (Y - L3 * qSin(P*M_PI / 180)) / (2*L1);

    Q = qSqrt((1 / (Xb * Xb + Zb * Zb)) - 1);


    p1 = qAtan2(Zb + Q * Xb, Xb - Q * Zb) * (180 / M_PI);
    p2 = qAtan2(Zb - Q * Xb, Xb + Q * Zb) * (180 / M_PI);
    t1 = p1-90; // back to degrees
    t2 = p2-t1; // back to degrees
    t3 = P-p2;  // back to degrees

//    double offset = qAbs(angle[0] * 180 / M_PI);
//    angle[0] = angle[0] * M_PI / 180; //back to radians
    angle[1] = ((t1 + 90) * 2 * M_PI) / 360;  // back to radians
    angle[2] = ((t2 - 90) * 2 * M_PI) / 360;  // back to radians
    angle[3] = ((t3) * 2 * M_PI) / 360;  // back to radians

    #define MAX_ALFA 146.3

    /*
    //kinematyka v2
        double c;
        double tmp_c;
        double tmp_angle;
        double tmp_val;
        int rotated;
        double dS5;
        double dS4;
        double dS3;
        double zS5;
        double zS4;
        double zS3;
        double P1_angle;
        double P2_angle;
double P3_angle;

        c = qSqrt(X*X + Y*Y);
                if (Y<0)
                    rotated = -1;
                else
                    rotated = 1;

                // S3, S4, S5
                if (c<12){
                    pout->S3 = 0;
                    pout->S4 = 255;
                    pout->S5 = round( 1.42 * MAX_ALFA );
                } else if (c>36){
                    pout->S3 = 128;
                    pout->S4 = 128;
                    pout->S5 = 0;
                } else{
                    tmp_c = round(c-12);
                    pout->S3 = round(5.33333 * tmp_c);
                    pout->S4 = 255-pout->S3;
                    pout->S5 = round((1.42 * MAX_ALFA) - (((1.42 * MAX_ALFA)/24) * tmp_c));
                }

                // S6
                if (X != 0){
                    tmp_angle = qAtan2((double)Y*rotated, (double)X);
                    //obrot dla uzyskania kierunku
                    pout->S6 = round(255 - (tmp_angle * (255/M_PI)));
                }else
                    pout->S6 = 128;

                // odwrocenie
                if (Y<0){
                    pout->S6 = 255 - pout->S6;
                    pout->S5 = 255 - pout->S5;
                    pout->S4 = 255 - pout->S4;
                    pout->S3 = 255 - pout->S3;
                }

                // obliczanie wysokosci Z
                zS5 = zS4 = zS3 = 0;
                c = qSqrt(c*c + Z*Z);
                dS5 = (128 - pout->S5)/36.0;
                dS4 = (128 - pout->S4)/36.0;
                dS3 = (128 - pout->S3)/36.0;
                for (int i=0; i<Z; i++){
                    zS5 += dS5;
                    zS4 += dS4;
                    zS3 += dS3;
                }
                pout->S5 = round(pout->S5+zS5);
                pout->S4 = round(pout->S4+zS4);
                pout->S3 = round(pout->S3+zS3);


                // kinematyka prosta z moodla
                angle[1] = ((pout->S5-128)*M_PI/256)+(M_PI/2);
                angle[2] = -(pout->S4-128)*(M_PI/256);
                angle[3] = (pout->S3-128)*(M_PI/256);
                angle[0] = 0;
                */


    // FORWARD KINEMATICS
    R0[0][0] = qCos(angle[0]);
    R0[0][2] = -qSin(angle[0]);
    R0[1][1] = 1;
    R0[2][0] = -qSin(angle[0]);
    R0[2][2] = qCos(angle[0]);
    R0[3][3] = 1;

    R1[0][0] = qCos(angle[1]);
    R1[0][1] = -qSin(angle[1]);
    R1[1][0] = qSin(angle[1]);
    R1[1][1] = qCos(angle[1]);
    R1[2][2] = 1;
    R1[3][3] = 1;

    R2[0][0] = qCos(angle[2]);
    R2[0][1] = -qSin(angle[2]);
    R2[1][0] = qSin(angle[2]);
    R2[1][1] = qCos(angle[2]);
    R2[2][2] = 1;
    R2[3][3] = 1;

    R3[0][0] = qCos(angle[3]);
    R3[0][1] = -qSin(angle[3]);
    R3[1][0] = qSin(angle[3]);
    R3[1][1] = qCos(angle[3]);
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

    qDebug("");
    qDebug("%f", segments[0].angle*180/M_PI);
    qDebug("%f", segments[1].angle*180/M_PI);
    qDebug("%f", segments[2].angle*180/M_PI);
    qDebug("%f", segments[3].angle*180/M_PI);

    for(int i=0; i<4; i++)
    {
        if(qIsNaN(angle[i]) )
            return false;
    }

    if(!checkValidity(P1))
        return false;
    if(!checkValidity(P2))
        return false;
    if(!checkValidity(P3))
        return false;

    for(int i=0; i<4; i++)
    {
        segments[i].angle =angle[i];
    }

    segments[0].end.set(P1[0][3], P1[2][3], P1[1][3]);
    segments[1].end.set(P2[0][3], P2[2][3], P2[1][3]);
    segments[2].end.set(P3[0][3], P3[2][3], P3[1][3]);


    angles[0] = (segments[0].angle*180/M_PI);
    angles[1] = (segments[1].angle*180/M_PI);
    angles[2] = (segments[2].angle*180/M_PI);
    angles[3] = (segments[3].angle*180/M_PI);


    int S6 = clamp(128 - (segments[0].angle*180/M_PI)*(255/SV0), 0, 255);
    int S5 = clamp(128 - (segments[1].angle*180/M_PI - 90)*(255/SV1), 0, 255);
    int S4 = clamp(128 - (segments[2].angle*180/M_PI)*(255/SV2), 0, 255);
    int S3 = clamp(128 - (segments[3].angle*180/M_PI)*(255/SV3), 0, 255);

    pout->S6 = S6;
    pout->S5 = S5;
    pout->S4 = S4;
    pout->S3 = S3;
    return true;
}
