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

struct Vec3 {
    float x, y, z;
    void set(float _x, float _y, float _z) {
        x= _x;
        y = _y;
        z = _z;
    }
};

struct Segment {
    Vec3 end;
    float angle;
};

class RobotSimulation
{
public:
    RobotSimulation(QGraphicsScene *scene, Servo *pout);
    bool calculatePosition(float x, float y, float z, float p, Servo *pout);
    void draw(QGraphicsScene *scene);
    void draw_td(QGraphicsScene *scene);
private:
    Segment segments[SEGMENTS];



};

#endif // ROBOTSIMULATION_H
