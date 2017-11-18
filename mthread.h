#ifndef MTHREAD_H
#define MTHREAD_H

#include <QThread>

class MThread : public QThread
{

    Q_OBJECT
protected:
    void run();
signals:
    void tick();
};

#endif // MTHREAD_H
