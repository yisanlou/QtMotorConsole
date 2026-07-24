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
    long targetTorque2 = 0;
    long targetTorque4 = 0;
    long grating1 = 0;
    long grating2 = 0;
    bool gratingSensor1Triggered = false;
    bool gratingSensor2Triggered = false;
    bool gratingSensor1Valid = false;
    bool gratingSensor2Valid = false;
};

struct ForceFeedbackStatus
{
    bool running = false;
    long targetPosition = 0;
    long plannedPosition = 0;
    long grating1Position = 0;
    long grating2Position = 0;
    long synchronizationError = 0;
    long axis2TorqueCommand = 0;
    long axis2ActualTorque = 0;
    long axis4TorqueCommand = 0;
    long axis4ActualTorque = 0;
};

struct ForceFeedbackTiming
{
    long long totalMaxUs = 0;
    long long gratingReadMaxUs = 0;
    long long actualTorqueReadMaxUs = 0;
    long long motorSampleMaxUs = 0;
    long long torqueWriteMaxUs = 0;
    long long updateMaxUs = 0;
};

#endif // MOTORTYPES_H
