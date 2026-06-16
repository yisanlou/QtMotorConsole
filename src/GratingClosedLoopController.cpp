#include "MotorInternal.h"
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kLoopPeriodUs = 5000;
constexpr long kPositionDeadbandPulse = 5;
constexpr double kFullStrokePulse = 9000000.0;
constexpr double kDefaultKpPermillePerPulse = 900.0 / kFullStrokePulse;
constexpr double kDefaultKiPermillePerPulseSecond = 0.00001;
constexpr double kDefaultKdPermillePerPulseSecond = 0.00002;
constexpr double kDefaultIntegralLimitPermille = 200.0;
constexpr double kDefaultFrictionCompensationPermille = 30.0;
constexpr double kDerivativeFilterLastWeight = 0.75;
constexpr double kDerivativeFilterCurrentWeight = 0.25;
constexpr long kDefaultTorqueLimitPermille = 900;

std::atomic<double> g_gratingLoopKp(kDefaultKpPermillePerPulse);
std::atomic<double> g_gratingLoopKi(kDefaultKiPermillePerPulseSecond);
std::atomic<double> g_gratingLoopKd(kDefaultKdPermillePerPulseSecond);
std::atomic<double> g_gratingLoopIntegralLimit(kDefaultIntegralLimitPermille);
std::atomic<double> g_gratingLoopFrictionCompensation(kDefaultFrictionCompensationPermille);
std::atomic_long g_gratingLoopTorqueLimit(kDefaultTorqueLimitPermille);

struct PidState
{
    double integralPermille = 0.0;
    double filteredVelocityPulsePerSecond = 0.0;
    long lastPosition = 0;
    bool hasLastPosition = false;
};

struct PidOutput
{
    long torque = 0;
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double friction = 0.0;
};

int AxisForGrating(int grating)
{
    return grating == 2 ? 4 : 2;
}

int CopyAxis2TorqueConfigToAxis4(const QString& modeName)
{
    int res = 0;
    int ret = 0;

    long axis2TorqueSource = ReadSdoLong(2, kIndexTorqueCommandSource, 0x00, &ret);
    if (ret == 0)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Copy axis2 PA25 to axis4"),
            g_MultiCard.MC_ECatSetSdoValue(4, kIndexTorqueCommandSource, 0x00, axis2TorqueSource, 2));
    }
    else
    {
        logMessage(QStringLiteral("%1：读取轴2 PA25失败，返回值=%2，轴4将继续使用当前PA25")
            .arg(modeName)
            .arg(ret));
    }

    long axis2MaxTorque = ReadSdoLong(2, 0x6072, 0x00, &ret);
    if (ret == 0 && axis2MaxTorque > 0)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Copy axis2 max torque 6072h to axis4"),
            g_MultiCard.MC_ECatSetSdoValue(4, 0x6072, 0x00, axis2MaxTorque, 2));
    }
    else if (ret != 0)
    {
        logMessage(QStringLiteral("%1：读取轴2 6072h失败，返回值=%2，轴4将继续使用当前6072h")
            .arg(modeName)
            .arg(ret));
    }

    return res;
}

int PositionDirectionForGrating(int grating)
{
    return 1;
}

int AxisTorqueDirectionForGrating(int grating)
{
    return grating == 1 ? -1 : 1;
}

double Sign(double value)
{
    if (value > 0.0)
        return 1.0;
    if (value < 0.0)
        return -1.0;
    return 0.0;
}

long ToLoopPosition(int grating, long uiPosition)
{
    return (long)PositionDirectionForGrating(grating) * uiPosition;
}

long ToAxisTorque(int grating, long loopTorque)
{
    return (long)AxisTorqueDirectionForGrating(grating) * loopTorque;
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

double TorqueLimit()
{
    double limit = (double)g_gratingLoopTorqueLimit.load();
    if (limit < 0.0)
        limit = -limit;
    if (limit <= 0.0)
        limit = (double)kDefaultTorqueLimitPermille;
    return limit;
}

bool PrepareLoopAxis(int grating)
{
    const int axis = AxisForGrating(grating);
    const QString modeName = QStringLiteral("Grating closed-loop debug");

    int res = 0;
    if (axis == 4)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Mirror axis2 torque config"), CopyAxis2TorqueConfigToAxis4(modeName));
        if (res != 0)
            return false;
    }

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

PidOutput CalculateLoopTorque(long targetPos, long currentPos, PidState* state)
{
    PidOutput output;
    long error = targetPos - currentPos;
    if (std::labs(error) <= kPositionDeadbandPulse)
    {
        state->integralPermille = 0.0;
        state->filteredVelocityPulsePerSecond = 0.0;
        state->lastPosition = currentPos;
        state->hasLastPosition = true;
        return output;
    }

    const double periodSecond = (double)kLoopPeriodUs / 1000000.0;
    double velocityPulsePerSecond = 0.0;
    if (state->hasLastPosition)
        velocityPulsePerSecond = ((double)currentPos - (double)state->lastPosition) / periodSecond;
    state->lastPosition = currentPos;
    state->hasLastPosition = true;

    state->filteredVelocityPulsePerSecond =
        state->filteredVelocityPulsePerSecond * kDerivativeFilterLastWeight +
        velocityPulsePerSecond * kDerivativeFilterCurrentWeight;

    output.p = g_gratingLoopKp.load() * (double)error;
    output.d = -g_gratingLoopKd.load() * state->filteredVelocityPulsePerSecond;

    double nextIntegral = ClampIntegral(state->integralPermille +
        g_gratingLoopKi.load() * (double)error * periodSecond);

    double rawTorque = output.p + nextIntegral + output.d;
    double limit = TorqueLimit();
    bool saturatedHigh = rawTorque > limit && error > 0;
    bool saturatedLow = rawTorque < -limit && error < 0;
    if (!saturatedHigh && !saturatedLow)
        state->integralPermille = nextIntegral;

    output.i = state->integralPermille;
    double pidTorque = output.p + output.i + output.d;
    double direction = Sign(pidTorque);
    if (direction == 0.0)
        direction = Sign((double)error);
    output.friction = direction * std::fabs(g_gratingLoopFrictionCompensation.load());
    output.torque = ClampTorque(RoundToLong(pidTorque + output.friction));
    return output;
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
    if (!g_uiGratingSampleValid.load())
    {
        ClearLoopTorque(grating);
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 closed-loop debug failed: UI grating sample is not ready.").arg(grating));
        return;
    }
    long uiPosition = grating == 2 ? g_uiGrating2.load() : g_uiGrating1.load();
    position = ToLoopPosition(grating, uiPosition);

    PidState pidState;
    bool firstCommandLogged = false;
    auto nextTime = std::chrono::steady_clock::now();
    while (g_gratingClosedLoopRunning[grating])
    {
        nextTime += std::chrono::microseconds(kLoopPeriodUs);

        if (!g_uiGratingSampleValid.load())
        {
            ClearLoopTorque(grating);
            pidState = PidState();
            logMessage(QStringLiteral("Grating %1 closed-loop debug: UI grating sample is not ready, torque cleared.").arg(grating));
            std::this_thread::sleep_until(nextTime);
            continue;
        }
        uiPosition = grating == 2 ? g_uiGrating2.load() : g_uiGrating1.load();
        position = ToLoopPosition(grating, uiPosition);

        PidOutput pid = CalculateLoopTorque(targetPos, position, &pidState);
        long axisTorque = ToAxisTorque(grating, pid.torque);
        if (!firstCommandLogged)
        {
            firstCommandLogged = true;
            logMessage(QStringLiteral("Grating %1 closed-loop debug first command: target=%2, uiPosition=%3, loopPosition=%4, error=%5, P=%6, I=%7, D=%8, F=%9, loopTorque=%10, axis%11Torque=%12")
                .arg(grating)
                .arg(targetPos)
                .arg(uiPosition)
                .arg(position)
                .arg(targetPos - position)
                .arg(pid.p, 0, 'f', 2)
                .arg(pid.i, 0, 'f', 2)
                .arg(pid.d, 0, 'f', 2)
                .arg(pid.friction, 0, 'f', 2)
                .arg(pid.torque)
                .arg(axis)
                .arg(axisTorque));
        }
        SetTargetTorque(axis, axisTorque, QStringLiteral("Grating closed-loop debug"), QStringLiteral("Set target torque 6071h"), false);
        g_MultiCard.MC_Update(1 << (axis - 1));

        std::this_thread::sleep_until(nextTime);
    }

    ClearLoopTorque(grating);
    DisableAxisDirect(axis);
}

}

void SetGratingClosedLoopConfig(double kp, double ki, double integralLimit, double kd, double frictionCompensation, long torqueLimit)
{
    g_gratingLoopKp = kp;
    g_gratingLoopKi = ki;
    g_gratingLoopKd = kd;
    g_gratingLoopIntegralLimit = integralLimit;
    g_gratingLoopFrictionCompensation = frictionCompensation;
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
