#ifndef MOTORTYPES_H
#define MOTORTYPES_H

struct MotorSample
{
    long pos1 = 0;
    long pos2 = 0;
    long pos3 = 0;
    long pos4 = 0;
    long vel1 = 0;
    long vel2 = 0;
    long vel3 = 0;
    long vel4 = 0;
    long torque1 = 0;
    long torque2 = 0;
    long torque3 = 0;
    long torque4 = 0;
    long grating1 = 0;
    long grating2 = 0;
    bool gratingSensor1Triggered = false;
    bool gratingSensor2Triggered = false;
    bool gratingSensor1Valid = false;
    bool gratingSensor2Valid = false;
};

#endif // MOTORTYPES_H
