#include "MotorInternal.h"

#include <algorithm>

#pragma execution_character_set("utf-8")

long ReadPosition(int axis)
{
    double pos = 0.0;
    if (g_MultiCard.MC_GetAxisEncPos(axis, &pos) == 0)
        return RoundToLong(pos);

    return ReadSdoLong(axis, 0x6064, 0x00);
}

long ReadVelocity(int axis)
{
    double velPulsePerMs = 0.0;
    if (g_MultiCard.MC_GetAxisEncVel(axis, &velPulsePerMs) == 0)
        return RoundToLong(velPulsePerMs * 1000.0);

    return ReadSdoLong(axis, 0x606C, 0x00);
}

long ReadTorque(int axis)
{
    long torque = ReadSdoLong(axis, 0x6077, 0x00);
    return (int16_t)torque;
}

bool ReadGratingEncoder(int encoderIndex, long* value, int* ret)
{
    double rawValue = 0.0;
    int localRet = g_MultiCard.MC_GetEncPos((short)encoderIndex, &rawValue, 1);
    if (ret)
        *ret = localRet;
    if (localRet != 0)
        return false;

    *value = RoundToLong(rawValue);
    return true;
}

bool ReadGratingPosition(int grating, long* value, int* ret)
{
    if ((grating != 1 && grating != 2) || !value)
        return false;

    const int encoderIndex = grating == 2 ? g_grating2EncoderIndex : 1;
    long rawValue = 0;
    if (!ReadGratingEncoder(encoderIndex, &rawValue, ret))
        return false;

    const long offset = grating == 2
        ? g_gratingOffset2.load()
        : g_gratingOffset1.load();
    const long rawMinusOffset = rawValue - offset;

    // Save, display and control all use this same zeroed common coordinate.
    *value = (long)GratingRawToCommonDirection(grating) * rawMinusOffset;
    return true;
}

bool ReadBothGratingPositions(long* grating1, long* grating2)
{
    if (!grating1 || !grating2 || g_grating2EncoderIndex < 1)
        return false;

    double encoderPosition[16] = {};
    if (g_grating2EncoderIndex > (int)(sizeof(encoderPosition) / sizeof(encoderPosition[0])))
        return false;

    const int ret = g_MultiCard.MC_GetEncPos(
        1,
        encoderPosition,
        (short)g_grating2EncoderIndex);
    if (ret != 0)
        return false;

    const long raw1 = RoundToLong(encoderPosition[0]);
    const long raw2 = RoundToLong(encoderPosition[g_grating2EncoderIndex - 1]);
    *grating1 = (long)GratingRawToCommonDirection(1) *
        (raw1 - g_gratingOffset1.load());
    *grating2 = (long)GratingRawToCommonDirection(2) *
        (raw2 - g_gratingOffset2.load());
    return true;
}

bool CaptureForceFeedbackSample(long grating1,
    long grating2,
    long axis2ActualTorque,
    long axis4ActualTorque,
    long axis2TorqueCommand,
    long axis4TorqueCommand)
{
    int activeAxisCount = std::max(0, std::min(kMaxAxisCount, g_activeAxisCount.load()));
    if (activeAxisCount <= 0)
        return false;

    MotorSample sample;
    {
        std::lock_guard<std::mutex> lock(g_forceFeedbackSampleMutex);
        if (g_forceFeedbackSampleValid)
            sample = g_forceFeedbackSample;
    }

    double encPos[kMaxAxisCount] = {};
    double encVelPulsePerMs[kMaxAxisCount] = {};
    const bool motorFeedbackValid =
        g_MultiCard.MC_GetAxisEncPos(1, encPos, (short)activeAxisCount) == 0 &&
        g_MultiCard.MC_GetAxisEncVel(1, encVelPulsePerMs, (short)activeAxisCount) == 0;
    if (motorFeedbackValid)
    {
        long* positions[kMaxAxisCount] = {
            &sample.pos1, &sample.pos2, &sample.pos3, &sample.pos4
        };
        long* velocities[kMaxAxisCount] = {
            &sample.vel1, &sample.vel2, &sample.vel3, &sample.vel4
        };
        for (int i = 0; i < activeAxisCount; ++i)
        {
            *positions[i] = RoundToLong(encPos[i]);
            *velocities[i] = RoundToLong(encVelPulsePerMs[i] * 1000.0);
        }
    }

    sample.torque1 = g_sampleTorqueCache[0].load();
    sample.torque2 = axis2ActualTorque;
    sample.torque3 = g_sampleTorqueCache[2].load();
    sample.torque4 = axis4ActualTorque;
    sample.targetTorque2 = axis2TorqueCommand;
    sample.targetTorque4 = axis4TorqueCommand;
    sample.grating1 = grating1;
    sample.grating2 = grating2;

    {
        std::lock_guard<std::mutex> lock(g_forceFeedbackSampleMutex);
        g_forceFeedbackSample = sample;
        g_forceFeedbackSampleValid = true;
    }
    return motorFeedbackValid;
}

bool ReadForceFeedbackSample(MotorSample* sample)
{
    if (!sample)
        return false;

    std::lock_guard<std::mutex> lock(g_forceFeedbackSampleMutex);
    if (!g_forceFeedbackSampleValid)
        return false;
    *sample = g_forceFeedbackSample;
    return true;
}

MotorSample ReadFastSample()
{
    MotorSample sample;
    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
        return sample;
    if (activeAxisCount > kMaxAxisCount)
        activeAxisCount = kMaxAxisCount;

    double encPos[kMaxAxisCount] = {};
    double encVelPulsePerMs[kMaxAxisCount] = {};

    if (g_MultiCard.MC_GetAxisEncPos(1, encPos, (short)activeAxisCount) == 0)
    {
        sample.pos1 = RoundToLong(encPos[0]);
        sample.pos2 = RoundToLong(encPos[1]);
        sample.pos3 = RoundToLong(encPos[2]);
        sample.pos4 = RoundToLong(encPos[3]);
    }
    else
    {
        if (activeAxisCount >= 1)
            sample.pos1 = ReadSdoLong(1, 0x6064, 0x00);
        if (activeAxisCount >= 2)
            sample.pos2 = ReadSdoLong(2, 0x6064, 0x00);
        if (activeAxisCount >= 3)
            sample.pos3 = ReadSdoLong(3, 0x6064, 0x00);
        if (activeAxisCount >= 4)
            sample.pos4 = ReadSdoLong(4, 0x6064, 0x00);
    }

    if (g_MultiCard.MC_GetAxisEncVel(1, encVelPulsePerMs, (short)activeAxisCount) == 0)
    {
        sample.vel1 = RoundToLong(encVelPulsePerMs[0] * 1000.0);
        sample.vel2 = RoundToLong(encVelPulsePerMs[1] * 1000.0);
        sample.vel3 = RoundToLong(encVelPulsePerMs[2] * 1000.0);
        sample.vel4 = RoundToLong(encVelPulsePerMs[3] * 1000.0);
    }
    else
    {
        if (activeAxisCount >= 1)
            sample.vel1 = ReadSdoLong(1, 0x606C, 0x00);
        if (activeAxisCount >= 2)
            sample.vel2 = ReadSdoLong(2, 0x606C, 0x00);
        if (activeAxisCount >= 3)
            sample.vel3 = ReadSdoLong(3, 0x606C, 0x00);
        if (activeAxisCount >= 4)
            sample.vel4 = ReadSdoLong(4, 0x606C, 0x00);
    }

    if (g_nextSampleTorqueAxis > activeAxisCount)
        g_nextSampleTorqueAxis = 1;

    int torqueRet = 0;
    long torque = ReadSdoLong(g_nextSampleTorqueAxis, 0x6077, 0x00, &torqueRet);
    if (torqueRet == 0)
        g_sampleTorqueCache[g_nextSampleTorqueAxis - 1] = (int16_t)torque;

    g_nextSampleTorqueAxis++;
    if (g_nextSampleTorqueAxis > activeAxisCount)
        g_nextSampleTorqueAxis = 1;

    int gratingRet = 0;
    long gratingValue = 0;
    if (ReadGratingPosition(1, &gratingValue, &gratingRet))
    {
        sample.grating1 = gratingValue;
    }
    else if (!g_grating1ReadErrorLogged)
    {
        g_grating1ReadErrorLogged = true;
        logMessage(QStringLiteral("光栅尺1：MC_GetEncPos(1)读取失败，返回值=%1").arg(gratingRet));
    }

    if (ReadGratingPosition(2, &gratingValue, &gratingRet))
    {
        sample.grating2 = gratingValue;
        if (gratingValue == g_grating2LastValue)
            g_grating2UnchangedCount++;
        else
        {
            g_grating2LastValue = gratingValue;
            g_grating2UnchangedCount = 0;
        }
    }
    else if (!g_grating2ReadErrorLogged)
    {
        g_grating2ReadErrorLogged = true;
        sample.grating2 = g_grating2LastValue;
        g_grating2UnchangedCount++;
        logMessage(QStringLiteral("光栅尺2：MC_GetEncPos(%1)读取失败，返回值=%2")
            .arg(g_grating2EncoderIndex)
            .arg(gratingRet));
    }
    else
    {
        sample.grating2 = g_grating2LastValue;
        g_grating2UnchangedCount++;
    }

    sample.gratingSensor1Valid = ReadGratingSensorTriggeredInternal(1, &sample.gratingSensor1Triggered);
    sample.gratingSensor2Valid = ReadGratingSensorTriggeredInternal(2, &sample.gratingSensor2Triggered);

    if (activeAxisCount >= 1)
    {
        sample.torque1 = g_sampleTorqueCache[0].load();
    }
    if (activeAxisCount >= 2)
    {
        sample.torque2 = g_sampleTorqueCache[1].load();
    }
    if (activeAxisCount >= 3)
    {
        sample.torque3 = g_sampleTorqueCache[2].load();
    }
    if (activeAxisCount >= 4)
    {
        sample.torque4 = g_sampleTorqueCache[3].load();
    }
    return sample;
}
