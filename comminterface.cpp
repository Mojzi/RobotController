#include "comminterface.h"

CommInterface::CommInterface()
{
#ifdef Q_OS_WIN
    hCommDev = CreateFileA("COM5",GENERIC_READ | GENERIC_WRITE, 0, NULL,CREATE_ALWAYS, 0,NULL);
    if(hCommDev != INVALID_HANDLE_VALUE)
    {
        dcb.DCBlength = sizeof(dcb);
        GetCommState(hCommDev, &dcb);
        dcb.BaudRate = CBR_9600;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.ByteSize = 8;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fAbortOnError = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fErrorChar = FALSE;
        dcb.fNull = FALSE;
        SetCommState(hCommDev, &dcb);
        Enabled = TRUE;
    }
    else
    {
        Enabled = FALSE;
        if(hCommDev == INVALID_HANDLE_VALUE)
        MessageBoxA(NULL,"Niewlasciwa nazwa portu lub port jest aktywny.","Error!",MB_OK);
    }
#endif
}

CommInterface::~CommInterface()
{
#ifdef Q_OS_WIN
    CloseHandle(hCommDev);
#endif
}

bool CommInterface::Send(Servo data)
{
#ifdef Q_OS_WIN
    DWORD NumberOfBytesWritten;
    DWORD fdwEvtMask;
    GetCommMask(hCommDev, &fdwEvtMask);
    SetCommMask(hCommDev, EV_TXEMPTY);
    Buffer_O[0]=255;
    Buffer_O[1]=0;
    Buffer_O[2]=data.S1;
    Buffer_O[3]=255;
    Buffer_O[4]=1;
    Buffer_O[5]=data.S2;
    Buffer_O[6]=255;
    Buffer_O[7]=2;
    Buffer_O[8]=data.S3;
    Buffer_O[9]=255;
    Buffer_O[10]=3;
    Buffer_O[11]=data.S4;
    Buffer_O[12]=255;
    Buffer_O[13]=4;
    Buffer_O[14]=data.S5;
    Buffer_O[15]=255;
    Buffer_O[16]=5;
    Buffer_O[17]=data.S6;
    if(WriteFile(hCommDev, &Buffer_O[0], 18, &NumberOfBytesWritten, NULL) > 0)
    {
        WaitCommEvent(hCommDev, &fdwEvtMask, NULL);
        return TRUE;
    }
    else
    {
        return FALSE;
    }
#endif
}
