#include "comminterface.h"

CommInterface::CommInterface()
{

    foreach (const QSerialPortInfo info, QSerialPortInfo::availablePorts())
    {
      if(info.manufacturer() == "FTDI" && info.description() == "USB Serial Port") serialPort.setPortName(info.portName());

    }
    serialPort.setBaudRate(QSerialPort::Baud9600);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    if(serialPort.open(QIODevice::ReadWrite))
    {
        enabled = true;
    }
    else
    {
        enabled = false;
        error =serialPort.errorString().toStdString();
    }
}

CommInterface::~CommInterface()
{
    serialPort.close();
}

bool CommInterface::send(Servo data)
{
    buffer_O[0]=255;
    buffer_O[1]=0;
    buffer_O[2]=data.S1;
    buffer_O[3]=255;
    buffer_O[4]=1;
    buffer_O[5]=data.S2;
    buffer_O[6]=255;
    buffer_O[7]=2;
    buffer_O[8]=data.S3;
    buffer_O[9]=255;
    buffer_O[10]=3;
    buffer_O[11]=data.S4;
    buffer_O[12]=255;
    buffer_O[13]=4;
    buffer_O[14]=data.S5;
    buffer_O[15]=255;
    buffer_O[16]=5;
    buffer_O[17]=data.S6;
    QByteArray buffer = QByteArray((char*)buffer_O, 18);
    if(serialPort.write(buffer))
    {
        serialPort.flush();
        serialPort.waitForBytesWritten(1000);
        return true;
    }
    else
    {
        return false;
    }
}
