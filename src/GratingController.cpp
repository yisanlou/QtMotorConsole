#include "MotorInternal.h"
#include <chrono>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {
constexpr short kGratingSensorCardIndex = 0; // 主模块；扩展 IO 模块从 1 开始
constexpr long kGratingZeroFastSpeed = 3000000;
constexpr long kGratingZeroSlowSpeed = 300000;
constexpr int kGratingZeroPollMs = 5;
constexpr int kGratingZeroSearchTimeoutMs = 30000;
constexpr unsigned short kGratingSensorTriggeredRawLevel = 0;

short GratingSensorBitIndex(int grating)
{
    return (short)(grating == 2 ? 31 : 30); // 光栅1 -> X30，光栅2 -> X31
}

QString GratingSensorName(int grating)
{
    return QStringLiteral("X%1").arg(GratingSensorBitIndex(grating));
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

bool ReadGratingSensorTriggeredInternal(int grating, bool* triggered, unsigned short* level)
{
    if (grating != 1 && grating != 2)
        return false;

    unsigned short currentLevel = 0;
    int ret = g_MultiCard.MC_GetExtDiBit(kGratingSensorCardIndex, GratingSensorBitIndex(grating), &currentLevel);
    if (ret != 0)
        return false;

    if (level)
        *level = currentLevel;
    if (triggered)
        *triggered = (currentLevel == kGratingSensorTriggeredRawLevel);

    return true;
}

bool ReadGratingSensorTriggered(int grating, bool* triggered)
{
    return ReadGratingSensorTriggeredInternal(grating, triggered);
}

static bool ReadGratingRawValue(int grating, long* value)
{
    int encoderIndex = (grating == 2) ? g_grating2EncoderIndex : 1;
    return ReadGratingEncoder(encoderIndex, value);
}

static void GratingZeroWorker(int grating)
{
    const int axis = (grating == 2) ? 4 : 2;
    // Search toward the switch in negative common direction, then leave the
    // switch in positive common direction. This guarantees both zeroed
    // grating positions increase when moving away from home.
    const long forwardSpeed =
        -(long)CommonToAxisDirection(axis) * kGratingZeroFastSpeed;
    const long reverseSpeed =
        (long)CommonToAxisDirection(axis) * kGratingZeroSlowSpeed;
    const QString modeName = QStringLiteral("光栅尺%1归零").arg(grating);
    const QString sensorName = GratingSensorName(grating);

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

    bool initialTriggered = false;
    unsigned short initialLevel = 0;
    if (!ReadGratingSensorTriggeredInternal(grating, &initialTriggered, &initialLevel))
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：读取%2失败，归零已停止。")
            .arg(modeName)
            .arg(sensorName));
        return;
    }

    bool sensorTriggered = initialTriggered;
    unsigned short triggerLevel = initialLevel;
    if (!sensorTriggered)
    {
        if (SetGratingZeroVelocity(axis, forwardSpeed, modeName, QStringLiteral("Set fast search velocity 60FFh")) != 0)
        {
            StopGratingZeroAxis(axis, modeName);
            g_gratingZeroRunning = false;
            logMessage(modeName + QStringLiteral("：高速搜索启动失败。"));
            return;
        }

        logMessage(QStringLiteral("%1：开始，电机%2以%3 pulse/s搜索%4触发状态，初始输入值=%5。")
            .arg(modeName)
            .arg(axis)
            .arg(forwardSpeed)
            .arg(sensorName)
            .arg(initialLevel));

        auto nextTime = std::chrono::steady_clock::now();
        auto searchStart = nextTime;
        while (g_gratingZeroRunning)
        {
            nextTime += std::chrono::milliseconds(kGratingZeroPollMs);
            std::this_thread::sleep_until(nextTime);

            bool currentTriggered = false;
            unsigned short currentLevel = 0;
            if (!ReadGratingSensorTriggeredInternal(grating, &currentTriggered, &currentLevel))
            {
                StopGratingZeroAxis(axis, modeName);
                g_gratingZeroRunning = false;
                logMessage(QStringLiteral("%1：搜索过程中读取%2失败，偏置未改变。")
                    .arg(modeName)
                    .arg(sensorName));
                return;
            }

            if (currentTriggered)
            {
                sensorTriggered = true;
                triggerLevel = currentLevel;
                break;
            }

            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nextTime - searchStart).count();
            if (elapsedMs >= kGratingZeroSearchTimeoutMs)
                break;
        }
    }
    else
    {
        logMessage(QStringLiteral("%1：开始时%2已处于触发状态，直接反向退出传感器。")
            .arg(modeName)
            .arg(sensorName));
    }

    if (!g_gratingZeroRunning)
    {
        StopGratingZeroAxis(axis, modeName);
        logMessage(QStringLiteral("%1：已取消，偏置未改变。").arg(modeName));
        return;
    }

    if (!sensorTriggered)
    {
        StopGratingZeroAxis(axis, modeName);
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：未检测到%2触发状态，偏置未改变。")
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
    auto nextTime = std::chrono::steady_clock::now();
    auto searchStart = nextTime;
    while (g_gratingZeroRunning)
    {
        nextTime += std::chrono::milliseconds(kGratingZeroPollMs);
        std::this_thread::sleep_until(nextTime);

        bool currentTriggered = true;
        unsigned short currentLevel = triggerLevel;
        if (!ReadGratingSensorTriggeredInternal(grating, &currentTriggered, &currentLevel))
        {
            StopGratingZeroAxis(axis, modeName);
            g_gratingZeroRunning = false;
            logMessage(QStringLiteral("%1：反向过程中读取%2失败，偏置未改变。")
                .arg(modeName)
                .arg(sensorName));
            return;
        }

        if (!currentTriggered)
        {
            sensorReleased = true;
            break;
        }

        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nextTime - searchStart).count();
        if (elapsedMs >= kGratingZeroSearchTimeoutMs)
            break;
    }

    StopGratingZeroAxis(axis, modeName);

    if (!g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("%1：已取消，偏置未改变。").arg(modeName));
        return;
    }

    if (!sensorReleased)
    {
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：反向退出超时，%2仍处于触发状态，偏置未改变。")
            .arg(modeName)
            .arg(sensorName));
        return;
    }

    long zeroPoint = 0;
    if (!ReadGratingRawValue(grating, &zeroPoint))
    {
        g_gratingZeroRunning = false;
        logMessage(QStringLiteral("%1：读取释放边沿光栅原始值失败，偏置未改变。").arg(modeName));
        return;
    }
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
