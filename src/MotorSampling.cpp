#include "MotorInternal.h"

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

static void ProbeGrating2Encoder(long grating1Value)
{
    int probeIndex = g_nextGrating2ProbeIndex;
    for (int i = 0; i < kMaxGratingEncoderIndex; i++)
    {
        if (probeIndex > kMaxGratingEncoderIndex)
            probeIndex = 2;
        if (probeIndex != 1 && probeIndex != g_grating2EncoderIndex)
            break;
        probeIndex++;
    }

    long probeValue = 0;
    if (ReadGratingEncoder(probeIndex, &probeValue))
    {
        if (g_gratingProbeHasValue[probeIndex] &&
            probeValue != g_gratingProbeLastValue[probeIndex] &&
            probeValue != grating1Value)
        {
            g_grating2EncoderIndex = probeIndex;
            g_grating2ReadErrorLogged = false;
            g_grating2UnchangedCount = 0;
            logMessage(QStringLiteral("光栅尺2：自动切换到编码器通道%1读取").arg(probeIndex));
        }

        g_gratingProbeLastValue[probeIndex] = probeValue;
        g_gratingProbeHasValue[probeIndex] = true;
    }

    g_nextGrating2ProbeIndex = probeIndex + 1;
    if (g_nextGrating2ProbeIndex > kMaxGratingEncoderIndex)
        g_nextGrating2ProbeIndex = 2;
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

    long gratingValue = 0;
    int gratingRet = 0;
    if (ReadGratingEncoder(1, &gratingValue, &gratingRet))
    {
        sample.grating1 = gratingValue - g_gratingOffset1.load();
    }
    else if (!g_grating1ReadErrorLogged)
    {
        g_grating1ReadErrorLogged = true;
        logMessage(QStringLiteral("光栅尺1：MC_GetEncPos(1)读取失败，返回值=%1").arg(gratingRet));
    }

    if (ReadGratingEncoder(g_grating2EncoderIndex, &gratingValue, &gratingRet))
    {
        long grating2Raw = gratingValue;
        sample.grating2 = grating2Raw - g_gratingOffset2.load();
        if (grating2Raw == g_grating2LastValue)
            g_grating2UnchangedCount++;
        else
        {
            g_grating2LastValue = grating2Raw;
            g_grating2UnchangedCount = 0;
        }
    }
    else if (!g_grating2ReadErrorLogged)
    {
        g_grating2ReadErrorLogged = true;
        sample.grating2 = g_grating2LastValue - g_gratingOffset2.load();
        g_grating2UnchangedCount++;
        logMessage(QStringLiteral("光栅尺2：MC_GetEncPos(%1)读取失败，返回值=%2")
            .arg(g_grating2EncoderIndex)
            .arg(gratingRet));
    }
    else
    {
        sample.grating2 = g_grating2LastValue - g_gratingOffset2.load();
        g_grating2UnchangedCount++;
    }

    if (g_grating2UnchangedCount >= 40)
        ProbeGrating2Encoder(sample.grating1);

    sample.gratingSensor1Valid = ReadGratingSensorTriggeredInternal(1, &sample.gratingSensor1Triggered);
    sample.gratingSensor2Valid = ReadGratingSensorTriggeredInternal(2, &sample.gratingSensor2Triggered);

    if (activeAxisCount >= 1)
    {
        sample.torque1 = g_sampleTorqueCache[0];
    }
    if (activeAxisCount >= 2)
    {
        sample.torque2 = g_sampleTorqueCache[1];
    }
    if (activeAxisCount >= 3)
    {
        sample.torque3 = g_sampleTorqueCache[2];
    }
    if (activeAxisCount >= 4)
    {
        sample.torque4 = g_sampleTorqueCache[3];
    }
    return sample;
}
