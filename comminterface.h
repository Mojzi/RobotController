#ifndef COMMINTERFACE_H
#define COMMINTERFACE_H

#include <QApplication>
#include <QSerialPort>
#include <QSerialPortInfo>

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
    QSerialPort serialPort;
    bool send(Servo);
    bool enabled;
    std::string error;
private:
    BYTE buffer_O[18];
};
#endif // COMMINTERFACE_H
