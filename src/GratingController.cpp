#include "MotorInternal.h"
#include <chrono>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {
constexpr short kGratingSensorCardIndex = 0; // 主模块；扩展 IO 模块从 1 开始
constexpr long kGratingZeroFastSpeed = 100000;
constexpr long kGratingZeroSlowSpeed = 10000;
constexpr int kGratingZeroPollMs = 5;
constexpr int kGratingZeroSearchTimeoutMs = 30000;

short GratingSensorBitIndex(int grating)
{
    return (short)(grating - 1); // 光栅1 -> IO1(bit0)，光栅2 -> IO2(bit1)
}

bool ReadGratingSensorLevel(int grating, unsigned short* level)
{
    return g_MultiCard.MC_GetExtDiBit(kGratingSensorCardIndex, GratingSensorBitIndex(grating), level) == 0;
}

int SetGratingZeroVelocity(int axis, long speed, const QString& modeName, const QString& stepName)
{
    int ret = g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, speed, 4);
    if (ret != 0)
    {
        logMessage(QStringLiteral("%1：%2失败，返回值=%3")
            .arg(modeName)
            .arg(stepName)
            .arg(ret));
        return ret;
    }

    ret = g_MultiCard.MC_Update(1 << (axis - 1));
    if (ret != 0)
    {
        logMessage(QStringLiteral("%1：%2 Update失败，返回值=%3")
            .arg(modeName)
            .arg(stepName)
            .arg(ret));
    }

    return ret;
}

void StopGratingZeroAxis(int axis, const QString& modeName)
{
    SetGratingZeroVelocity(axis, 0, modeName, QStringLiteral("Stop target velocity 60FFh"));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

bool PrepareGratingZeroAxis(int axis, const QString& modeName)
{
    int res = 0;
    res = AddStepResult(res, modeName, QStringLiteral("Clear target velocity 60FFh"), g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4));
    res = AddStepResult(res, modeName, QStringLiteral("Update zero command"), g_MultiCard.MC_Update(1 << (axis - 1)));
    if (res != 0)
        return false;

    if (!SwitchAxisMode(axis, kModeCsv, modeName))
        return false;
    if (!EnableAxisCiA402(axis, modeName))
        return false;

    g_axisMode[axis] = AxisModeVelocity;
    return true;
}
}

static long ReadGratingRawValue(int grating)
{
    long value = 0;
    int encoderIndex = (grating == 2) ? g_grating2EncoderIndex : 1;
    ReadGratingEncoder(encoderIndex, &value);
    return value;
}

static void GratingZeroWorker(int grating)
{
    const int axis = (grating == 2) ? 4 : 2;
    const long forwardSpeed = (grating == 2) ? -kGratingZeroFastSpeed : kGratingZeroFastSpeed;
    const long reverseSpeed = (grating == 2) ? kGratingZeroSlowSpeed : -kGratingZeroSlowSpeed;
    const QString modeName = QStringLiteral("光栅尺%1归零").arg(grating);
    const QString sensorName = QStringLiteral("主模块IO%1").arg(grating);

    if (!IsCardOpened())
    {
        logMessage(modeName + QStringLiteral("：控制卡未打开。"));
        g_gratingZeroRunning = false;
        return;
    }

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();
    StopAllGratingClosedLoop();

    if (!PrepareGratingZeroAxis(axis, modeName))
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(modeName + QStringLiteral("：启动失败。"));
        return;
    }

    unsigned short initialLevel = 0;
    if (!ReadGratingSensorLevel(grating, &initialLevel))
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：读取%2失败，归零已停止。")
            .arg(modeName)
            .arg(sensorName));
        return;
    }

    if (SetGratingZeroVelocity(axis, forwardSpeed, modeName, QStringLiteral("Set fast search velocity 60FFh")) != 0)
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(modeName + QStringLiteral("：高速搜索启动失败。"));
        return;
    }

    logMessage(QStringLiteral("%1：开始，电机%2以%3 pulse/s搜索%4电平变化，初始电平=%5。")
        .arg(modeName)
        .arg(axis)
        .arg(forwardSpeed)
        .arg(sensorName)
        .arg(initialLevel));

    bool sensorTriggered = false;
    unsigned short triggerLevel = initialLevel;
    auto nextTime = std::chrono::steady_clock::now();
    auto searchStart = nextTime;
    while (g_gratingZeroRunning)
    {
        nextTime += std::chrono::milliseconds(kGratingZeroPollMs);
        std::this_thread::sleep_until(nextTime);

        unsigned short currentLevel = initialLevel;
        if (!ReadGratingSensorLevel(grating, &currentLevel))
        {
            StopGratingZeroAxis(axis, modeName);
            g_gratingZeroRunning = false;
            logMessage(QStringLiteral("%1：搜索过程中读取%2失败，偏置未改变。")
                .arg(modeName)
                .arg(sensorName));
            return;
        }

        if (currentLevel != initialLevel)
        {
            sensorTriggered = true;
            triggerLevel = currentLevel;
            break;
        }

        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nextTime - searchStart).count();
        if (elapsedMs >= kGratingZeroSearchTimeoutMs)
            break;
    }

    if (!sensorTriggered)
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：未检测到%2电平变化，偏置未改变。")
            .arg(modeName)
            .arg(sensorName));
        return;
    }

    if (SetGratingZeroVelocity(axis, reverseSpeed, modeName, QStringLiteral("Set slow release velocity 60FFh")) != 0)
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(modeName + QStringLiteral("：低速反向启动失败，偏置未改变。"));
        return;
    }

    logMessage(QStringLiteral("%1：%2已触发，电平=%3；电机%4以%5 pulse/s反向退出传感器。")
        .arg(modeName)
        .arg(sensorName)
        .arg(triggerLevel)
        .arg(axis)
        .arg(reverseSpeed));

    bool sensorReleased = false;
    nextTime = std::chrono::steady_clock::now();
    searchStart = nextTime;
    while (g_gratingZeroRunning)
    {
        nextTime += std::chrono::milliseconds(kGratingZeroPollMs);
        std::this_thread::sleep_until(nextTime);

        unsigned short currentLevel = triggerLevel;
        if (!ReadGratingSensorLevel(grating, &currentLevel))
        {
            StopGratingZeroAxis(axis, modeName);
            g_gratingZeroRunning = false;
            logMessage(QStringLiteral("%1：反向过程中读取%2失败，偏置未改变。")
                .arg(modeName)
                .arg(sensorName));
            return;
        }

        if (currentLevel != triggerLevel)
        {
            sensorReleased = true;
            break;
        }

        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nextTime - searchStart).count();
        if (elapsedMs >= kGratingZeroSearchTimeoutMs)
            break;
    }

    StopGratingZeroAxis(axis, modeName);

    if (!sensorReleased)
    {
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：反向退出超时，%2仍处于触发电平，偏置未改变。")
            .arg(modeName)
            .arg(sensorName));
        return;
    }

    long zeroPoint = ReadGratingRawValue(grating);
    if (grating == 2)
        g_gratingOffset2 = zeroPoint;
    else
        g_gratingOffset1 = zeroPoint;

    g_gratingZeroRunning = false;
    logMessage(QStringLiteral("%1：完成，%2释放边沿原始值=%3，偏置已设置为%4，当前点显示为0。")
        .arg(modeName)
        .arg(sensorName)
        .arg(zeroPoint)
        .arg(zeroPoint));
}

void StartGratingZero(int grating)
{
    if (grating != 1 && grating != 2)
        return;

    if (g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("光栅尺归零正在执行，请等待完成。"));
        return;
    }

    if (g_gratingZeroThread.joinable())
        g_gratingZeroThread.join();

    g_gratingZeroRunning = true;
    g_gratingZeroThread = std::thread([grating]() {
        GratingZeroWorker(grating);
        });
}
