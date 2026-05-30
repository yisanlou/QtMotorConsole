#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <QFile>
#include <QTextStream>
#include <QDateTime>

void StartRecord();
void StopRecord();
void RecordDataSample(long long timeIndex,
    long pos1,
    long vel1,
    long torque1,
    long pos2,
    long vel2,
    long torque2,
    long pos3,
    long vel3,
    long torque3,
    long pos4,
    long vel4,
    long torque4,
    long grating1,
    long grating2);



#endif
