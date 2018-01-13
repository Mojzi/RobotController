#ifndef ROBOTSIMULATION_H
#define ROBOTSIMULATION_H
#include <vector>
#include <math.h>
#include <qmath.h>
#include <QGraphicsScene>
#include <QMainWindow>
#include "comminterface.h"

#define SEGMENTS 3

#define FIRST_SEGMENT_LEN 4.75
#define SECOND_SEGMENT_LEN 4.75
#define THIRD_SEGMENT_LEN 5.75

struct Vec2 {
    float x, y;
    void set(float _x, float _y) {
        x= _x;
        y = _y;
    }
};

struct Segment {
    Vec2 end;
};

class RobotSimulation
{
public:
    RobotSimulation(QGraphicsScene *scene, Servo *pout);
    void kinematics(float x, float y, float z, float p, Servo *pout);
    void draw(QGraphicsScene *scene);
    void draw_td(QGraphicsScene *scene, float x, float y, float z);
    void getPositionAsServo(Servo *pout);
    void setValuesFromServo(Servo *pout);
private:
    Segment segments[SEGMENTS];
    Segment segments_td[SEGMENTS];
    QGraphicsLineItem *arms[SEGMENTS];
    QGraphicsRectItem *joints[SEGMENTS];

    QGraphicsLineItem *arms_td[SEGMENTS];
    QGraphicsRectItem *joints_td[SEGMENTS];
};

#endif // ROBOTSIMULATION_H
