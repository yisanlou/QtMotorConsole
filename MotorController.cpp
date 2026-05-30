#include "MotorController.h"
#include <QDebug>
#include <QDateTime>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

// 全局变量定义
MultiCard g_MultiCard;
std::atomic_bool g_bFollowRunning(false);
std::atomic_bool g_cardOpened(false);
std::atomic_bool g_forceFeedbackStopRequested(false);
std::atomic_long g_forceFeedbackTargetVel(0);
std::atomic_long g_forceFeedbackCurrentVel(0);
std::atomic_llong g_forceFeedbackExecMaxUs(0);
std::thread g_forceFeedbackThread;

QTextBrowser* g_logWidget = nullptr;  // 先初始化为 nullptr

enum AxisWorkMode
{
    AxisModeUnknown = 0,
    AxisModePosition,
    AxisModeVelocity,
    AxisModeTorque
};

static const int kMaxAxisCount = 4;
static AxisWorkMode g_axisMode[kMaxAxisCount + 1] = {};
static std::atomic_int g_activeAxisCount(0);
static const unsigned short kModeCsp = 0x08;
static const unsigned short kModeCsv = 0x09;
static const unsigned short kModeCst = 0x0A;
static const int kIndexTorqueCommandSource = 0x2019; // PA25
static const int kTorqueCommandSourceBus6080 = 3;
static const unsigned short kControlShutdown = 0x0006;
static const unsigned short kControlSwitchOn = 0x0007;
static const unsigned short kControlEnableOperation = 0x000F;
static const unsigned short kControlFaultReset = 0x0080;
static std::atomic_bool g_cardClosing(false);
static long g_sampleTorqueCache[kMaxAxisCount] = {};
static int g_nextSampleTorqueAxis = 1;
static bool g_grating1ReadErrorLogged = false;
static bool g_grating2ReadErrorLogged = false;
static const int kMaxGratingEncoderIndex = 8;
static int g_grating2EncoderIndex = 2;
static int g_nextGrating2ProbeIndex = 2;
static long g_grating2LastValue = 0;
static int g_grating2UnchangedCount = 0;
static long g_gratingProbeLastValue[kMaxGratingEncoderIndex + 1] = {};
static bool g_gratingProbeHasValue[kMaxGratingEncoderIndex + 1] = {};

static void ResetFastSampleCache()
{
    for (int i = 0; i < kMaxAxisCount; i++)
        g_sampleTorqueCache[i] = 0;

    g_nextSampleTorqueAxis = 1;
    g_grating1ReadErrorLogged = false;
    g_grating2ReadErrorLogged = false;
    g_grating2EncoderIndex = 2;
    g_nextGrating2ProbeIndex = 2;
    g_grating2LastValue = 0;
    g_grating2UnchangedCount = 0;
    for (int i = 0; i <= kMaxGratingEncoderIndex; i++)
    {
        g_gratingProbeLastValue[i] = 0;
        g_gratingProbeHasValue[i] = false;
    }
}

static bool IsValidAxis(int axis)
{
    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
        activeAxisCount = kMaxAxisCount;

    return axis >= 1 && axis <= activeAxisCount;
}

static bool CanAccessEtherCAT()
{
    return g_cardOpened.load();
}

static long ReadSdoLong(int axis, int index, int subIndex, int* ret = nullptr)
{
    long value = 0;
    int localRet = g_MultiCard.MC_ECatGetSdoValue(axis, index, subIndex, &value);
    if (ret)
        *ret = localRet;
    return value;
}

long ReadOperationMode(int axis)
{
    if (!IsValidAxis(axis))
        return 0;

    return ReadSdoLong(axis, 0x6061, 0x00);
}

static int SetSdoWithLog(int axis, int index, int subIndex, long value, short len, const QString& name)
{
    int ret = g_MultiCard.MC_ECatSetSdoValue(axis, index, subIndex, value, len);
    if (ret != 0)
    {
        logMessage(QStringLiteral("轴%1：写%2(0x%3:%4=%5)失败，返回值=%6")
            .arg(axis)
            .arg(name)
            .arg(index, 4, 16, QChar('0'))
            .arg(subIndex)
            .arg(value)
            .arg(ret));
    }

    return ret;
}

static long ClampInt16(long value)
{
    if (value > 32767)
        return 32767;
    if (value < -32768)
        return -32768;
    return value;
}

static long RoundToLong(double value)
{
    return (long)std::llround(value);
}

static short ReadStatusWord(int axis, int* ret = nullptr)
{
    short statusWord = 0;
    int localRet = g_MultiCard.MC_ECatGetStatusWord(axis, &statusWord);
    if (ret)
        *ret = localRet;
    return statusWord;
}

static bool WaitAxisOnline(int axis, long* supportedModes, long* displayMode, short* statusWord)
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

static bool IsOperationEnabled(short statusWord)
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

static int WriteControlWord(int axis, unsigned short value, const QString& stepName)
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

static bool EnableAxisCiA402(int axis, const QString& modeName)
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

static bool SwitchAxisMode(int axis, unsigned short mode, const QString& modeName)
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

static void PrepareAxisForDisable(int axis)
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

static void DisableAxisDirect(int axis)
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

static void DisableAllAxesForClose()
{
    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
        activeAxisCount = kMaxAxisCount;

    for (int axis = 1; axis <= activeAxisCount; axis++)
        DisableAxisDirect(axis);
}

static long SmoothVelocityStep(long currentVel, long targetVel, long step)
{
    if (currentVel < targetVel)
    {
        currentVel += step;
        if (currentVel > targetVel)
            currentVel = targetVel;
    }
    else if (currentVel > targetVel)
    {
        currentVel -= step;
        if (currentVel < targetVel)
            currentVel = targetVel;
    }

    return currentVel;
}

// ----------------- 全局日志函数 -----------------
void logMessage(const QString& msg)
{
    QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString fullMsg = QString("[%1] %2").arg(timeStr, msg);

    if (g_logWidget)
        g_logWidget->append(fullMsg);  // 中文正常显示

    qDebug().noquote() << fullMsg;
}

static int AddStepResult(int total, const QString& modeName, const QString& stepName, int ret)
{
    if (ret != 0)
        logMessage(QStringLiteral("%1：%2失败，返回值=%3").arg(modeName, stepName).arg(ret));

    return total + ret;
}

static bool PrepareTorqueModeCommandSource(int axis, const QString& modeName)
{
    int ret = 0;
    long source = ReadSdoLong(axis, kIndexTorqueCommandSource, 0x00, &ret);
    if (ret == 0 && source == kTorqueCommandSourceBus6080)
        return true;

    int writeRet = g_MultiCard.MC_ECatSetSdoValue(axis, kIndexTorqueCommandSource, 0x00, kTorqueCommandSourceBus6080, 2);
    if (writeRet != 0)
    {
        logMessage(QStringLiteral("%1：PA25(0x2019)当前值=%2，设置为总线力矩来源3失败，返回值=%3；将继续尝试使能，请在驱动器参数中确认PA25=3")
            .arg(modeName)
            .arg(source)
            .arg(writeRet));
        return true;
    }

    Sleep(20);
    long verifySource = ReadSdoLong(axis, kIndexTorqueCommandSource, 0x00, &ret);
    if (ret == 0 && verifySource != kTorqueCommandSourceBus6080)
    {
        logMessage(QStringLiteral("%1：PA25(0x2019)写入后读回=%2，期望=3；将继续尝试使能，如力矩不响应请在驱动器面板保存PA25=3并重启")
            .arg(modeName)
            .arg(verifySource));
        return true;
    }

    return true;
}

static int SetTargetTorque(int axis, long torque, const QString& modeName, const QString& stepName, bool verifyReadBack = true)
{
    long targetTorque = ClampInt16(torque);
    if (targetTorque != torque)
    {
        logMessage(QStringLiteral("%1：目标力矩%2超出I16范围，已限幅为%3")
            .arg(modeName)
            .arg(torque)
            .arg(targetTorque));
    }

    int limitRet = 0;
    long maxTorque = ReadSdoLong(axis, 0x6072, 0x00, &limitRet);
    if (limitRet == 0 && maxTorque > 0)
    {
        if (targetTorque > maxTorque)
            targetTorque = maxTorque;
        else if (targetTorque < -maxTorque)
            targetTorque = -maxTorque;
    }

    int ret = g_MultiCard.MC_ECatSetSdoValue(axis, 0x6071, 0x00, targetTorque, 2);
    if (ret != 0)
    {
        logMessage(QStringLiteral("%1：%2失败，返回值=%3")
            .arg(modeName)
            .arg(stepName)
            .arg(ret));
        return ret;
    }

    if (verifyReadBack)
    {
        int readRet = 0;
        long readBack = (int16_t)ReadSdoLong(axis, 0x6071, 0x00, &readRet);
        if (readRet == 0 && readBack != targetTorque)
        {
            logMessage(QStringLiteral("%1：6071h写入%2但读回%3，请确认该轴RxPDO已映射6071h")
                .arg(modeName)
                .arg(targetTorque)
                .arg(readBack));
        }
    }

    return 0;
}

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

static bool ReadGratingEncoder(int encoderIndex, long* value, int* ret = nullptr)
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
        sample.grating1 = gratingValue;
    }
    else if (!g_grating1ReadErrorLogged)
    {
        g_grating1ReadErrorLogged = true;
        logMessage(QStringLiteral("光栅尺1：MC_GetEncPos(1)读取失败，返回值=%1").arg(gratingRet));
    }

    if (ReadGratingEncoder(g_grating2EncoderIndex, &gratingValue, &gratingRet))
    {
        sample.grating2 = gratingValue;
        if (sample.grating2 == g_grating2LastValue)
            g_grating2UnchangedCount++;
        else
        {
            g_grating2LastValue = sample.grating2;
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

    if (g_grating2UnchangedCount >= 40)
        ProbeGrating2Encoder(sample.grating1);

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

void OpenCard()
{
    char PCip[] = "192.168.0.200";
    char cardIp[] = "192.168.0.1";
    int res = 0;

    if (g_cardOpened.load())
    {
        logMessage(QStringLiteral("控制卡已打开，先执行一次安全关闭。"));
        CloseCard();
        Sleep(200);
    }
    else
    {
        // 清理上一次打开失败后可能残留在库内部的句柄。
        g_MultiCard.MC_Close();
        Sleep(100);
    }

    g_cardClosing = false;
    res = g_MultiCard.MC_Open(1, PCip, 60000, cardIp, 60000);
    if (res != 0)
    {
        logMessage(QStringLiteral("Open card failed，返回值=%1。").arg(res));
        return;
    }
    logMessage(QStringLiteral("Open card success."));

    g_MultiCard.MC_SetCommuTimer(3);
    Sleep(100);
    res = g_MultiCard.MC_ECatInit();
    Sleep(600);
    if (res != 0)
    {
        logMessage(QStringLiteral("EtherCAT init failed，返回值=%1。").arg(res));
        CloseCard();
        return;
    }

    short slaveCount = 0;
    res = g_MultiCard.MC_ECatGetSlaveCount(&slaveCount);
    if (res != 0 || slaveCount <= 0)
    {
        logMessage(QStringLiteral("Servo count check failed，返回值=%1，检测到数量=%2。").arg(res).arg(slaveCount));
        CloseCard();
        return;
    }

    int activeAxisCount = slaveCount > kMaxAxisCount ? kMaxAxisCount : slaveCount;
    g_activeAxisCount = activeAxisCount;
    ResetFastSampleCache();

    if (slaveCount > kMaxAxisCount)
    {
        logMessage(QStringLiteral("检测到%1个从站，程序最多启用前%2个轴。").arg(slaveCount).arg(kMaxAxisCount));
    }
    else
    {
        logMessage(QStringLiteral("检测到%1个从站，启用%2个轴。").arg(slaveCount).arg(activeAxisCount));
    }

    for (int axis = 1; axis <= activeAxisCount; axis++)
    {
        int loadRet = g_MultiCard.MC_ECatLoadPDOConfig(axis);
        if (loadRet != 0)
            logMessage(QStringLiteral("轴%1：PDO加载失败，返回值=%2").arg(axis).arg(loadRet));
        res += loadRet;
    }

    if (res != 0)
    {
        logMessage(QStringLiteral("PDO init failed，返回值=%1。").arg(res));
        CloseCard();
        return;
    }

    if (!ConfigServo())
    {
        logMessage(QStringLiteral("Servo config failed."));
        CloseCard();
        return;
    }

    g_cardOpened = true;
    logMessage(QStringLiteral("Servo config success."));
}

void CloseCard()
{
    if (g_cardClosing.exchange(true))
        return;

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();

    DisableAllAxesForClose();
    Sleep(100);

    g_cardOpened = false;
    for (int axis = 1; axis <= kMaxAxisCount; axis++)
        g_axisMode[axis] = AxisModeUnknown;
    g_activeAxisCount = 0;
    ResetFastSampleCache();

    g_MultiCard.MC_Close();
    Sleep(200);
    g_cardClosing = false;
    logMessage(QStringLiteral("Card closed."));
}

void StartForceFeedback(long vel)
{
    if (g_bFollowRunning)
    {
        SetForceFeedbackVel(vel);
        return;
    }

    if (g_forceFeedbackThread.joinable())
        g_forceFeedbackThread.join();

    g_forceFeedbackStopRequested = false;
    g_forceFeedbackCurrentVel = 0;
    g_forceFeedbackTargetVel = vel;
    g_bFollowRunning = true;
    g_forceFeedbackThread = std::thread([vel]() {
        ForceFeedback(vel);
        });

    logMessage(QStringLiteral("Force feedback started."));
}

void SetForceFeedbackVel(long vel)
{
    g_forceFeedbackTargetVel = vel;
    logMessage(QStringLiteral("Force feedback velocity updated."));
}

void StopForceFeedback()
{
    g_forceFeedbackTargetVel = 0;
    g_forceFeedbackStopRequested = true;

    if (g_forceFeedbackThread.joinable())
        g_forceFeedbackThread.join();
    else
    {
        g_bFollowRunning = false;
        DisableAxisDirect(1);
        DisableAxisDirect(2);
    }

    logMessage(QStringLiteral("Force feedback stopped."));
}

void ForceFeedback(long vel)
{
    const int periodMs = 10;
    const long maxTorque = 100;
    const long minTorque = -100;
    const long velocityStep = 50;
    long lastTorque2 = 0;
    int res = 0;

    if (!SwitchAxisMode(1, kModeCsv, QStringLiteral("力反馈主轴速度模式")) ||
        !SwitchAxisMode(2, kModeCst, QStringLiteral("力反馈从轴力矩模式")) ||
        !PrepareTorqueModeCommandSource(2, QStringLiteral("力反馈从轴力矩模式")) ||
        !EnableAxisCiA402(1, QStringLiteral("力反馈主轴")) ||
        !EnableAxisCiA402(2, QStringLiteral("力反馈从轴")))
    {
        g_bFollowRunning = false;
        DisableAxisDirect(1);
        DisableAxisDirect(2);
        return;
    }

    g_axisMode[1] = AxisModeVelocity;
    g_axisMode[2] = AxisModeTorque;
    res += g_MultiCard.MC_ECatSetSdoValue(1, 0x60FF, 0x00, 0, 4);
    res += SetTargetTorque(2, 0, QStringLiteral("力反馈从轴力矩模式"), QStringLiteral("Clear target torque 6071h"));
    res += g_MultiCard.MC_Update(0xFF);

    if (res != 0)
    {
        g_bFollowRunning = false;
        DisableAxisDirect(1);
        DisableAxisDirect(2);
        return;
    }

    auto nextTime = std::chrono::steady_clock::now();
    while (g_bFollowRunning)
    {
        auto execStart = std::chrono::steady_clock::now();
        nextTime += std::chrono::milliseconds(periodMs);

        long oldVel = g_forceFeedbackCurrentVel.load();
        long targetVel = g_forceFeedbackTargetVel.load();
        long currentVel = SmoothVelocityStep(oldVel, targetVel, velocityStep);

        if (currentVel != oldVel)
        {
            g_forceFeedbackCurrentVel = currentVel;
            g_MultiCard.MC_ECatSetSdoValue(1, 0x60FF, 0x00, currentVel, 4);
            g_MultiCard.MC_Update(1 << 0);
        }

        if (g_forceFeedbackStopRequested && currentVel == 0)
        {
            g_bFollowRunning = false;
            break;
        }

        long rawTorque = 0;
        g_MultiCard.MC_ECatGetSdoValue(1, 0x6077, 0x00, &rawTorque);
        long torque1 = (int16_t)rawTorque;
        long torque2 = (long)(torque1 * 0.8 + lastTorque2 * 0.2);

        if (torque2 > maxTorque)
            torque2 = maxTorque;
        if (torque2 < minTorque)
            torque2 = minTorque;

        lastTorque2 = torque2;
        SetTargetTorque(2, -torque2, QStringLiteral("力反馈从轴力矩模式"), QStringLiteral("Set target torque 6071h"), false);
        g_MultiCard.MC_Update(0xFF);

        auto execEnd = std::chrono::steady_clock::now();
        long long execUs = std::chrono::duration_cast<std::chrono::microseconds>(execEnd - execStart).count();
        long long oldMaxUs = g_forceFeedbackExecMaxUs.load();
        while (execUs > oldMaxUs && !g_forceFeedbackExecMaxUs.compare_exchange_weak(oldMaxUs, execUs))
        {
        }

        std::this_thread::sleep_until(nextTime);
    }

    g_MultiCard.MC_ECatSetSdoValue(1, 0x60FF, 0x00, 0, 4);
    g_MultiCard.MC_Update(1 << 0);
    g_forceFeedbackCurrentVel = 0;
    g_forceFeedbackTargetVel = 0;
    g_forceFeedbackStopRequested = false;
    DisableAxisDirect(1);
    DisableAxisDirect(2);
}
void PositionModeMove(int axis, long targetPos, long speed)
{
    const QString modeName = QStringLiteral("位置模式");

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();

    if (!IsValidAxis(axis))
    {
        logMessage(QStringLiteral("位置模式：电机编号无效！"));
        return;
    }

    if (speed < 0)
        speed = -speed;

    int res = 0;
    if (!SwitchAxisMode(axis, kModeCsp, modeName))
        return;
    if (!EnableAxisCiA402(axis, modeName))
        return;

    res = AddStepResult(res, modeName, QStringLiteral("Switch trap planner"), g_MultiCard.MC_PrfTrap(axis));

    TTrapPrm trapPrm;
    trapPrm.acc = 0.5;
    trapPrm.dec = 0.5;
    trapPrm.velStart = 0;
    trapPrm.smoothTime = 0;

    res = AddStepResult(res, modeName, QStringLiteral("Set trap parameters"), g_MultiCard.MC_SetTrapPrm(axis, &trapPrm));
    res = AddStepResult(res, modeName, QStringLiteral("Set target position 607Ah"), g_MultiCard.MC_SetPos(axis, targetPos));
    res = AddStepResult(res, modeName, QStringLiteral("Set target speed"), g_MultiCard.MC_SetVel(axis, speed));
    res = AddStepResult(res, modeName, QStringLiteral("Update command"), g_MultiCard.MC_Update(1 << (axis - 1)));

    if (res == 0)
    {
        g_axisMode[axis] = AxisModePosition;
        logMessage(QStringLiteral("位置模式启动成功！"));
    }
    else
        logMessage(QStringLiteral("位置模式启动失败！"));
}

void VelocityModeMove(int axis, long speed)
{
    const QString modeName = QStringLiteral("速度模式");

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();

    if (!IsValidAxis(axis))
    {
        logMessage(QStringLiteral("速度模式：电机编号无效！"));
        return;
    }

    int res = 0;
    bool alreadyReady = ReadOperationMode(axis) == kModeCsv && IsOperationEnabled(ReadStatusWord(axis));
    if (!alreadyReady)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Clear target velocity 60FFh"), g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4));
        res = AddStepResult(res, modeName, QStringLiteral("Update zero command"), g_MultiCard.MC_Update(1 << (axis - 1)));
        if (res != 0)
        {
            logMessage(QStringLiteral("速度模式启动失败！"));
            return;
        }
    }

    if (!SwitchAxisMode(axis, kModeCsv, modeName))
        return;
    if (!EnableAxisCiA402(axis, modeName))
        return;

    res = AddStepResult(res, modeName, QStringLiteral("Set target velocity 60FFh"), g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, speed, 4));
    res = AddStepResult(res, modeName, QStringLiteral("Update command"), g_MultiCard.MC_Update(1 << (axis - 1)));

    if (res == 0)
    {
        g_axisMode[axis] = AxisModeVelocity;
        logMessage(QStringLiteral("速度模式启动成功！"));
    }
    else
        logMessage(QStringLiteral("速度模式启动失败！"));
}

void TorqueModeSet(int axis, long torque)
{
    const QString modeName = QStringLiteral("力矩模式");

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();

    if (!IsValidAxis(axis))
    {
        logMessage(QStringLiteral("力矩模式：电机编号无效！"));
        return;
    }

    int res = 0;
    bool alreadyReady = ReadOperationMode(axis) == kModeCst && IsOperationEnabled(ReadStatusWord(axis));
    if (!alreadyReady)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Clear target torque 6071h"), SetTargetTorque(axis, 0, modeName, QStringLiteral("Clear target torque 6071h")));
        res = AddStepResult(res, modeName, QStringLiteral("Update zero command"), g_MultiCard.MC_Update(1 << (axis - 1)));
        if (res != 0)
        {
            logMessage(QStringLiteral("力矩模式设置失败！"));
            return;
        }
    }

    if (!SwitchAxisMode(axis, kModeCst, modeName))
        return;
    if (!PrepareTorqueModeCommandSource(axis, modeName))
        return;
    if (!EnableAxisCiA402(axis, modeName))
        return;

    res = AddStepResult(res, modeName, QStringLiteral("Set target torque 6071h"), SetTargetTorque(axis, torque, modeName, QStringLiteral("Set target torque 6071h")));
    res = AddStepResult(res, modeName, QStringLiteral("Update command"), g_MultiCard.MC_Update(1 << (axis - 1)));

    if (res == 0)
    {
        g_axisMode[axis] = AxisModeTorque;
        logMessage(QStringLiteral("力矩模式设置成功！"));
    }
    else
        logMessage(QStringLiteral("力矩模式设置失败！"));
}

void DisableAxis(int axis)
{
    if (!IsValidAxis(axis))
        return;

    DisableAxisDirect(axis);
    logMessage(QStringLiteral("当前电机已失能！"));
}
bool ConfigServo()
{
    int res = 0;

    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
    {
        logMessage(QStringLiteral("Servo config failed：没有可用轴。"));
        return false;
    }

    for (int axis = 1; axis <= activeAxisCount; axis++)
    {
        long supportedModes = 0;
        long displayMode = 0;
        short statusWord = 0;

        if (!WaitAxisOnline(axis, &supportedModes, &displayMode, &statusWord))
        {
            logMessage(QStringLiteral("轴%1：EtherCAT从站未就绪，支持模式=0x%2，当前模式=0x%3，状态字=0x%4")
                .arg(axis)
                .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
                .arg((unsigned long)displayMode, 2, 16, QChar('0'))
                .arg((unsigned short)statusWord, 4, 16, QChar('0')));
            res |= 1;
            continue;
        }

        int modeRet = g_MultiCard.MC_ECatSetCtrlMode(axis, kModeCsp);
        if (modeRet != 0)
        {
            int directRet = g_MultiCard.MC_ECatSetSdoValue(axis, 0x6060, 0x00, kModeCsp, 1);
            if (directRet != 0)
            {
                logMessage(QStringLiteral("轴%1：设置初始CSP模式失败，控制卡返回=%2，直接写6060h返回=%3")
                    .arg(axis)
                    .arg(modeRet)
                    .arg(directRet));
                res |= modeRet;
            }
        }

        Sleep(30);
        displayMode = ReadOperationMode(axis);
        statusWord = ReadStatusWord(axis);

        logMessage(QStringLiteral("轴%1：EtherCAT就绪，支持模式=0x%2，当前模式=0x%3，状态字=0x%4")
            .arg(axis)
            .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
            .arg((unsigned long)displayMode, 2, 16, QChar('0'))
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
    }

    return res == 0;
}
bool IsCardOpened()
{
    return g_cardOpened.load() && !g_cardClosing.load();
}

long long ConsumeForceFeedbackExecMaxUs()
{
    return g_forceFeedbackExecMaxUs.exchange(0);
}
