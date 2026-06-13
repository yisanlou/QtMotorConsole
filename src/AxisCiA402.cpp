#include "MotorInternal.h"
#include <Windows.h>

#pragma execution_character_set("utf-8")

bool WaitAxisOnline(int axis, long* supportedModes, long* displayMode, short* statusWord)
{
    for (int retry = 0; retry < 30; retry++)
    {
        int modesRet = 0;
        int modeRet = 0;
        int statusRet = 0;
        long modes = ReadSdoLong(axis, 0x6502, 0x00, &modesRet);
        long mode = ReadSdoLong(axis, 0x6061, 0x00, &modeRet);
        short status = ReadStatusWord(axis, &statusRet);

        if (supportedModes)
            *supportedModes = modes;
        if (displayMode)
            *displayMode = mode;
        if (statusWord)
            *statusWord = status;

        if (modesRet == 0 && modeRet == 0 && statusRet == 0 && (modes != 0 || mode != 0 || status != 0))
            return true;

        Sleep(100);
    }

    return false;
}

static bool IsFaultState(short statusWord)
{
    return (statusWord & 0x0008) != 0;
}

bool IsOperationEnabled(short statusWord)
{
    return (statusWord & 0x006F) == 0x0027;
}

static bool IsSwitchedOn(short statusWord)
{
    return (statusWord & 0x006F) == 0x0023;
}

static bool IsAxisPlannerRunning(int axis)
{
    long cardStatus = 0;
    if (g_MultiCard.MC_GetSts(axis, &cardStatus) != 0)
        return false;

    return (cardStatus & AXIS_STATUS_RUNNING) != 0;
}

static void StopAxisPlanner(int axis)
{
    const long mask = 1 << (axis - 1);

    g_MultiCard.MC_Stop(mask, 0);
    g_MultiCard.MC_Update(mask);

    for (int i = 0; i < 50 && IsAxisPlannerRunning(axis); i++)
        Sleep(10);

    g_MultiCard.MC_ClrSts(axis);
}

static bool WaitStatus(int axis, bool (*predicate)(short), int timeoutMs)
{
    const int intervalMs = 10;
    for (int elapsed = 0; elapsed <= timeoutMs; elapsed += intervalMs)
    {
        if (predicate(ReadStatusWord(axis)))
            return true;
        Sleep(intervalMs);
    }

    return false;
}

int WriteControlWord(int axis, unsigned short value, const QString& stepName)
{
    int ret = SetSdoWithLog(axis, 0x6040, 0x00, value, 2, stepName);
    if (ret == 0)
        ret = g_MultiCard.MC_Update(1 << (axis - 1));
    return ret;
}

static void LogEnableDiagnostics(int axis, const QString& modeName, const QString& reason)
{
    int ret = 0;
    short statusWord = ReadStatusWord(axis, &ret);
    int statusRet = ret;
    long controlWord = ReadSdoLong(axis, 0x6040, 0x00, &ret);
    int controlRet = ret;
    long modeCommand = ReadSdoLong(axis, 0x6060, 0x00, &ret);
    int modeCommandRet = ret;
    long modeDisplay = ReadSdoLong(axis, 0x6061, 0x00, &ret);
    int modeDisplayRet = ret;
    long errorCode = ReadSdoLong(axis, 0x603F, 0x00, &ret);
    int errorRet = ret;
    long supportedModes = ReadSdoLong(axis, 0x6502, 0x00, &ret);
    int supportedRet = ret;
    long torqueSource = ReadSdoLong(axis, kIndexTorqueCommandSource, 0x00, &ret);
    int torqueSourceRet = ret;
    long targetTorque = ReadSdoLong(axis, 0x6071, 0x00, &ret);
    int targetTorqueRet = ret;
    long maxTorque = ReadSdoLong(axis, 0x6072, 0x00, &ret);
    int maxTorqueRet = ret;
    long cardStatus = 0;
    int cardRet = g_MultiCard.MC_GetSts(axis, &cardStatus);

    logMessage(QStringLiteral("%1：轴%2使能诊断(%3)：状态字=0x%4(ret=%5)，控制字=0x%6(ret=%7)，6060=0x%8(ret=%9)，6061=0x%10(ret=%11)，603F=0x%12(ret=%13)，6502=0x%14(ret=%15)，PA25=0x%16(ret=%17)，6071=%18(ret=%19)，6072=%20(ret=%21)，板卡状态=0x%22(ret=%23)")
        .arg(modeName)
        .arg(axis)
        .arg(reason)
        .arg((unsigned short)statusWord, 4, 16, QChar('0'))
        .arg(statusRet)
        .arg((unsigned short)controlWord, 4, 16, QChar('0'))
        .arg(controlRet)
        .arg((unsigned long)modeCommand, 2, 16, QChar('0'))
        .arg(modeCommandRet)
        .arg((unsigned long)modeDisplay, 2, 16, QChar('0'))
        .arg(modeDisplayRet)
        .arg((unsigned long)errorCode, 4, 16, QChar('0'))
        .arg(errorRet)
        .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
        .arg(supportedRet)
        .arg((unsigned long)torqueSource, 4, 16, QChar('0'))
        .arg(torqueSourceRet)
        .arg((long)(int16_t)targetTorque)
        .arg(targetTorqueRet)
        .arg(maxTorque)
        .arg(maxTorqueRet)
        .arg((unsigned long)cardStatus, 8, 16, QChar('0'))
        .arg(cardRet));
}

bool EnableAxisCiA402(int axis, const QString& modeName)
{
    short statusWord = ReadStatusWord(axis);
    if (IsOperationEnabled(statusWord))
        return true;

    if (IsFaultState(statusWord))
    {
        if (WriteControlWord(axis, kControlFaultReset, QStringLiteral("6040故障复位")) != 0)
            return false;
        Sleep(50);
    }

    if (WriteControlWord(axis, kControlShutdown, QStringLiteral("6040 Shutdown")) != 0)
        return false;
    Sleep(20);

    if (WriteControlWord(axis, kControlSwitchOn, QStringLiteral("6040 Switch on")) != 0)
        return false;
    if (!WaitStatus(axis, IsSwitchedOn, 300) && !IsOperationEnabled(ReadStatusWord(axis)))
    {
        statusWord = ReadStatusWord(axis);
        logMessage(QStringLiteral("%1：等待Switched on超时，状态字=0x%2")
            .arg(modeName)
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
    }

    int axisOnRet = g_MultiCard.MC_AxisOn(axis);
    if (axisOnRet != 0)
    {
        statusWord = ReadStatusWord(axis);
        logMessage(QStringLiteral("%1：MC_AxisOn返回=%2，状态字=0x%3，继续尝试6040 Enable operation")
            .arg(modeName)
            .arg(axisOnRet)
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
    }

    for (int retry = 0; retry < 5 && !IsOperationEnabled(ReadStatusWord(axis)); retry++)
    {
        if (WriteControlWord(axis, kControlEnableOperation, QStringLiteral("6040 Enable operation")) != 0)
            return false;
        Sleep(30);
    }

    if (!IsOperationEnabled(ReadStatusWord(axis)))
    {
        LogEnableDiagnostics(axis, modeName, QStringLiteral("第一次Enable operation后"));

        for (int retry = 0; retry < 10 && !IsOperationEnabled(ReadStatusWord(axis)); retry++)
        {
            g_MultiCard.MC_ECatSetSdoValue(axis, 0x6040, 0x00, kControlEnableOperation, 2);
            g_MultiCard.MC_Update(1 << (axis - 1));
            Sleep(50);
        }
    }

    if (!WaitStatus(axis, IsOperationEnabled, 300))
    {
        statusWord = ReadStatusWord(axis);
        long cardStatus = 0;
        g_MultiCard.MC_GetSts(axis, &cardStatus);
        LogEnableDiagnostics(axis, modeName, QStringLiteral("最终超时"));
        logMessage(QStringLiteral("%1：使能超时，状态字=0x%2，板卡状态=0x%3")
            .arg(modeName)
            .arg((unsigned short)statusWord, 4, 16, QChar('0'))
            .arg((unsigned long)cardStatus, 8, 16, QChar('0')));
        return false;
    }

    Sleep(120);
    return true;
}

bool SwitchAxisMode(int axis, unsigned short mode, const QString& modeName)
{
    if (!IsValidAxis(axis))
    {
        logMessage(modeName + QStringLiteral("：电机编号无效！"));
        return false;
    }

    long actualMode = ReadOperationMode(axis);
    short statusWord = ReadStatusWord(axis);
    if (actualMode == mode)
        return true;

    if (IsOperationEnabled(statusWord) || IsAxisPlannerRunning(axis))
    {
        StopAxisPlanner(axis);
        WriteControlWord(axis, kControlShutdown, QStringLiteral("6040切模式前失能"));
        Sleep(30);
    }

    int ret = g_MultiCard.MC_ECatSetCtrlMode(axis, mode);
    if (ret != 0)
    {
        int sdoRet = g_MultiCard.MC_ECatSetSdoValue(axis, 0x6060, 0x00, mode, 1);
        if (sdoRet != 0)
        {
            statusWord = ReadStatusWord(axis);
            logMessage(QStringLiteral("%1：设置控制模式0x%2失败，控制卡返回=%3，直接写6060h返回=%4，状态字=0x%5")
                .arg(modeName)
                .arg(mode, 2, 16, QChar('0'))
                .arg(ret)
                .arg(sdoRet)
                .arg((unsigned short)statusWord, 4, 16, QChar('0')));
            return false;
        }

        logMessage(QStringLiteral("%1：控制卡设置模式0x%2返回=%3，已改用6060h直接写入")
            .arg(modeName)
            .arg(mode, 2, 16, QChar('0'))
            .arg(ret));
    }

    Sleep(25);

    actualMode = ReadOperationMode(axis);
    if (actualMode != mode)
    {
        statusWord = ReadStatusWord(axis);
        logMessage(QStringLiteral("%1：模式显示值为0x%2，期望0x%3，状态字=0x%4；请确认驱动器支持该模式且允许总线切换")
            .arg(modeName)
            .arg(actualMode, 2, 16, QChar('0'))
            .arg(mode, 2, 16, QChar('0'))
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
        return false;
    }

    return true;
}

void PrepareAxisForDisable(int axis)
{
    const long mask = 1 << (axis - 1);

    StopAxisPlanner(axis);
    g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4);
    g_MultiCard.MC_ECatSetSdoValue(axis, 0x6071, 0x00, 0, 2);
    g_MultiCard.MC_Update(mask);

    if (g_axisMode[axis] == AxisModeTorque)
    {
        g_MultiCard.MC_ECatSetSdoValue(axis, 0x6071, 0x00, 0, 2);
        g_MultiCard.MC_Update(mask);
    }
    else if (g_axisMode[axis] == AxisModeVelocity)
    {
        g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4);
        g_MultiCard.MC_Update(mask);
    }

    for (int i = 0; i < 50; i++)
    {
        long vel = ReadVelocity(axis);
        if (vel < 10 && vel > -10)
            break;

        Sleep(10);
        if (g_axisMode[axis] == AxisModeVelocity)
        {
            g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4);
            g_MultiCard.MC_Update(mask);
        }
    }

    if (g_axisMode[axis] == AxisModePosition)
    {
        g_MultiCard.MC_ECatSetSdoValue(axis, 0x607A, 0x00, ReadPosition(axis), 4);
        g_MultiCard.MC_Update(mask);
    }

    Sleep(30);
}

void DisableAxisDirect(int axis)
{
    if (!IsValidAxis(axis) || !CanAccessEtherCAT())
        return;

    PrepareAxisForDisable(axis);
    WriteControlWord(axis, kControlSwitchOn, QStringLiteral("6040退出运行"));
    Sleep(20);
    WriteControlWord(axis, kControlShutdown, QStringLiteral("6040失能"));
    g_MultiCard.MC_AxisOff(axis);
    g_MultiCard.MC_ClrSts(axis);
    g_axisMode[axis] = AxisModeUnknown;
}

void DisableAllAxesForClose()
{
    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
        activeAxisCount = kMaxAxisCount;

    for (int axis = 1; axis <= activeAxisCount; axis++)
        DisableAxisDirect(axis);
}

