#ifndef COMMINTERFACE_H
#define COMMINTERFACE_H

#include <QApplication>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

typedef unsigned char BYTE;

typedef struct
{
    BYTE S1, S2, S3, S4, S5, S6;
}Servo;

typedef struct
{
    double S1, S2, S3, S4, S5, S6;
} ServoR;

class CommInterface
{
public:
    CommInterface();
    ~CommInterface();
    bool Send(Servo);
    bool Enabled;
private:
#ifdef Q_OS_WIN
    HANDLE hCommDev;
    DCB dcb;
#endif

    BYTE Buffer_O[18];
};
#endif // COMMINTERFACE_H
