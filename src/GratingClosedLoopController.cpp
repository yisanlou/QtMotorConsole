#include "MotorInternal.h"
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kLoopPeriodUs = 5000;
constexpr long kPositionDeadbandPulse = 5;
constexpr double kDefaultKpPermillePerPulse = 0.2;
constexpr double kDefaultKiPermillePerPulseSecond = 0.02;
constexpr double kDefaultIntegralLimitPermille = 500.0;
constexpr double kDefaultFrictionFeedForwardPermille = 0.0;
constexpr long kDefaultTorqueLimitPermille = 1000;

std::atomic<double> g_gratingLoopKp(kDefaultKpPermillePerPulse);
std::atomic<double> g_gratingLoopKi(kDefaultKiPermillePerPulseSecond);
std::atomic<double> g_gratingLoopIntegralLimit(kDefaultIntegralLimitPermille);
std::atomic<double> g_gratingLoopFrictionFeedForward(kDefaultFrictionFeedForwardPermille);
std::atomic_long g_gratingLoopTorqueLimit(kDefaultTorqueLimitPermille);

int AxisForGrating(int grating)
{
    return grating == 2 ? 4 : 2;
}

int EncoderForGrating(int grating)
{
    return grating == 2 ? g_grating2EncoderIndex : 1;
}

long OffsetForGrating(int grating)
{
    return grating == 2 ? g_gratingOffset2.load() : g_gratingOffset1.load();
}

double Sign(double value)
{
    if (value > 0.0)
        return 1.0;
    if (value < 0.0)
        return -1.0;
    return 0.0;
}

long ClampTorque(long torque)
{
    long limit = g_gratingLoopTorqueLimit.load();
    if (limit < 0)
        limit = -limit;
    if (limit <= 0)
        limit = kDefaultTorqueLimitPermille;

    if (torque > limit)
        return limit;
    if (torque < -limit)
        return -limit;
    return torque;
}

double ClampIntegral(double integral)
{
    double limit = std::fabs(g_gratingLoopIntegralLimit.load());
    if (limit <= 0.0)
        limit = kDefaultIntegralLimitPermille;

    if (integral > limit)
        return limit;
    if (integral < -limit)
        return -limit;
    return integral;
}

bool ReadLoopGrating(int grating, long* position)
{
    long rawValue = 0;
    if (!ReadGratingEncoder(EncoderForGrating(grating), &rawValue))
        return false;

    *position = rawValue - OffsetForGrating(grating);
    return true;
}

bool PrepareLoopAxis(int grating)
{
    const int axis = AxisForGrating(grating);
    const QString modeName = QStringLiteral("Grating closed-loop debug");

    int res = 0;
    res = AddStepResult(res, modeName, QStringLiteral("Clear target torque 6071h"),
        SetTargetTorque(axis, 0, modeName, QStringLiteral("Clear target torque 6071h"), false));
    res = AddStepResult(res, modeName, QStringLiteral("Update zero torque"),
        g_MultiCard.MC_Update(1 << (axis - 1)));
    if (res != 0)
        return false;

    if (!SwitchAxisMode(axis, kModeCst, modeName))
        return false;
    if (!PrepareTorqueModeCommandSource(axis, modeName))
        return false;
    if (!EnableAxisCiA402(axis, modeName))
        return false;

    g_axisMode[axis] = AxisModeTorque;
    return true;
}

long CalculateLoopTorque(long targetPos, long currentPos, double* integralPermille)
{
    long error = targetPos - currentPos;
    if (std::labs(error) <= kPositionDeadbandPulse)
    {
        *integralPermille = 0.0;
        return 0;
    }

    const double periodSecond = (double)kLoopPeriodUs / 1000000.0;
    double p = g_gratingLoopKp.load() * (double)error;
    *integralPermille = ClampIntegral(*integralPermille +
        g_gratingLoopKi.load() * (double)error * periodSecond);

    double torque = p + *integralPermille;
    double direction = Sign(torque);
    if (direction == 0.0)
        direction = Sign((double)error);
    torque += direction * g_gratingLoopFrictionFeedForward.load();

    return ClampTorque(RoundToLong(torque));
}

void ClearLoopTorque(int grating)
{
    const int axis = AxisForGrating(grating);
    SetTargetTorque(axis, 0, QStringLiteral("Grating closed-loop debug"), QStringLiteral("Clear target torque 6071h"), false);
    g_MultiCard.MC_Update(1 << (axis - 1));
}

void GratingClosedLoopWorker(int grating, long targetPos)
{
    const int axis = AxisForGrating(grating);
    if (!IsCardOpened())
    {
        logMessage(QStringLiteral("Grating %1 closed-loop debug failed: card is not open.").arg(grating));
        g_gratingClosedLoopRunning[grating] = false;
        return;
    }

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();
    if (g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("Grating %1 closed-loop debug rejected: grating zero is running.").arg(grating));
        g_gratingClosedLoopRunning[grating] = false;
        return;
    }
    if (g_gratingZeroThread.joinable())
        g_gratingZeroThread.join();

    if (!PrepareLoopAxis(grating))
    {
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 closed-loop debug failed: axis %2 preparation failed.").arg(grating).arg(axis));
        return;
    }

    long position = 0;
    if (!ReadLoopGrating(grating, &position))
    {
        ClearLoopTorque(grating);
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 closed-loop debug failed: grating read failed.").arg(grating));
        return;
    }

    double integralPermille = 0.0;
    auto nextTime = std::chrono::steady_clock::now();
    while (g_gratingClosedLoopRunning[grating])
    {
        nextTime += std::chrono::microseconds(kLoopPeriodUs);

        if (!ReadLoopGrating(grating, &position))
        {
            ClearLoopTorque(grating);
            integralPermille = 0.0;
            logMessage(QStringLiteral("Grating %1 closed-loop debug: read failed, torque cleared.").arg(grating));
            std::this_thread::sleep_until(nextTime);
            continue;
        }

        long torque = CalculateLoopTorque(targetPos, position, &integralPermille);
        SetTargetTorque(axis, torque, QStringLiteral("Grating closed-loop debug"), QStringLiteral("Set target torque 6071h"), false);
        g_MultiCard.MC_Update(1 << (axis - 1));

        std::this_thread::sleep_until(nextTime);
    }

    ClearLoopTorque(grating);
    DisableAxisDirect(axis);
}

}

void SetGratingClosedLoopConfig(double kp, double ki, double integralLimit, double frictionFeedForward, long torqueLimit)
{
    g_gratingLoopKp = kp;
    g_gratingLoopKi = ki;
    g_gratingLoopIntegralLimit = integralLimit;
    g_gratingLoopFrictionFeedForward = frictionFeedForward;
    g_gratingLoopTorqueLimit = torqueLimit;
}

void StartGratingClosedLoop(int grating, long targetPos)
{
    if (grating != 1 && grating != 2)
        return;

    if (g_gratingClosedLoopRunning[grating])
    {
        logMessage(QStringLiteral("Grating %1 closed-loop debug target updated requires restart; stop first.").arg(grating));
        return;
    }

    if (g_gratingClosedLoopThread[grating].joinable())
        g_gratingClosedLoopThread[grating].join();

    g_gratingClosedLoopRunning[grating] = true;
    g_gratingClosedLoopThread[grating] = std::thread([grating, targetPos]() {
        GratingClosedLoopWorker(grating, targetPos);
        });
}

void StopGratingClosedLoop(int grating)
{
    if (grating != 1 && grating != 2)
        return;

    g_gratingClosedLoopRunning[grating] = false;
    if (g_gratingClosedLoopThread[grating].joinable())
        g_gratingClosedLoopThread[grating].join();
}

void StopAllGratingClosedLoop()
{
    StopGratingClosedLoop(1);
    StopGratingClosedLoop(2);
}
