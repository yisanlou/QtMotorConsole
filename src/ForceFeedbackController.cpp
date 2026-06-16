#include "MotorInternal.h"
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kForceFeedbackPeriodUs = 5000; // 5 ms, 200 Hz
constexpr int kAxis2TorqueLoopAxis = 2;
constexpr int kAxis4TorqueFollowerAxis = 4;
constexpr long kPositionDeadbandPulse = 5;
constexpr long kTorqueLimitPermille = 1000;
constexpr double kAxis2KiPermillePerPulseSecond = 0.003;
constexpr double kAxis2IntegralLimitPermille = 120.0;
constexpr double kAxis2FrictionFeedForwardPermille = 0.0;
constexpr double kDefaultMitKpPermillePerPulse = 0.12;
constexpr double kDefaultMitKdPermillePerPulsePerSecond = 0.12;
constexpr double kAxis4Axis2TorqueFeedForwardScale = 1.0;

std::atomic<double> g_forceFeedbackMitKp(kDefaultMitKpPermillePerPulse);
std::atomic<double> g_forceFeedbackMitKd(kDefaultMitKdPermillePerPulsePerSecond);

int CopyAxis2TorqueConfigToAxis4(const QString& modeName)
{
    int res = 0;
    int ret = 0;

    long axis2TorqueSource = ReadSdoLong(kAxis2TorqueLoopAxis, kIndexTorqueCommandSource, 0x00, &ret);
    if (ret == 0)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Copy axis2 PA25 to axis4"),
            g_MultiCard.MC_ECatSetSdoValue(kAxis4TorqueFollowerAxis, kIndexTorqueCommandSource, 0x00, axis2TorqueSource, 2));
    }
    else
    {
        logMessage(QStringLiteral("%1：读取轴2 PA25失败，返回值=%2，轴4将继续使用当前PA25")
            .arg(modeName)
            .arg(ret));
    }

    long axis2MaxTorque = ReadSdoLong(kAxis2TorqueLoopAxis, 0x6072, 0x00, &ret);
    if (ret == 0 && axis2MaxTorque > 0)
    {
        res = AddStepResult(res, modeName, QStringLiteral("Copy axis2 max torque 6072h to axis4"),
            g_MultiCard.MC_ECatSetSdoValue(kAxis4TorqueFollowerAxis, 0x6072, 0x00, axis2MaxTorque, 2));
    }
    else if (ret != 0)
    {
        logMessage(QStringLiteral("%1：读取轴2 6072h失败，返回值=%2，轴4将继续使用当前6072h")
            .arg(modeName)
            .arg(ret));
    }

    return res;
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
    if (torque > kTorqueLimitPermille)
        return kTorqueLimitPermille;
    if (torque < -kTorqueLimitPermille)
        return -kTorqueLimitPermille;
    return torque;
}

double ClampIntegral(double integral)
{
    if (integral > kAxis2IntegralLimitPermille)
        return kAxis2IntegralLimitPermille;
    if (integral < -kAxis2IntegralLimitPermille)
        return -kAxis2IntegralLimitPermille;
    return integral;
}

long ForceFeedbackAxisMask()
{
    return (1 << (kAxis2TorqueLoopAxis - 1)) | (1 << (kAxis4TorqueFollowerAxis - 1));
}

int AxisTorqueDirection(int axis)
{
    return axis == kAxis2TorqueLoopAxis ? -1 : 1;
}

long ToAxisTorqueCommand(int axis, long loopTorque)
{
    return (long)AxisTorqueDirection(axis) * loopTorque;
}

long ToLoopTorqueFeedback(int axis, long axisTorque)
{
    return (long)AxisTorqueDirection(axis) * axisTorque;
}

bool ReadForceFeedbackGrating1(long* position)
{
    long rawValue = 0;
    if (!ReadGratingEncoder(1, &rawValue))
        return false;

    *position = g_gratingOffset1.load() - rawValue;
    return true;
}

bool ReadForceFeedbackGrating2(long* position)
{
    long rawValue = 0;
    if (!ReadGratingEncoder(g_grating2EncoderIndex, &rawValue))
        return false;

    *position = g_gratingOffset2.load() - rawValue;
    return true;
}

bool PrepareAxis2TorqueLoop()
{
    const QString modeName = QStringLiteral("Force feedback axis2 grating1 loop");
    int res = 0;
    res = AddStepResult(res, modeName, QStringLiteral("Clear target torque 6071h"),
        SetTargetTorque(kAxis2TorqueLoopAxis, 0, modeName, QStringLiteral("Clear target torque 6071h"), false));
    res = AddStepResult(res, modeName, QStringLiteral("Update zero torque"),
        g_MultiCard.MC_Update(1 << (kAxis2TorqueLoopAxis - 1)));
    if (res != 0)
        return false;

    if (!SwitchAxisMode(kAxis2TorqueLoopAxis, kModeCst, modeName))
        return false;
    if (!PrepareTorqueModeCommandSource(kAxis2TorqueLoopAxis, modeName))
        return false;
    if (!EnableAxisCiA402(kAxis2TorqueLoopAxis, modeName))
        return false;

    g_axisMode[kAxis2TorqueLoopAxis] = AxisModeTorque;
    return true;
}

bool PrepareAxis4TorqueFollower()
{
    const QString modeName = QStringLiteral("Force feedback axis4 torque follower");
    int res = 0;
    res = AddStepResult(res, modeName, QStringLiteral("Mirror axis2 torque config"), CopyAxis2TorqueConfigToAxis4(modeName));
    res = AddStepResult(res, modeName, QStringLiteral("Clear target torque 6071h"),
        SetTargetTorque(kAxis4TorqueFollowerAxis, 0, modeName, QStringLiteral("Clear target torque 6071h"), false));
    res = AddStepResult(res, modeName, QStringLiteral("Update zero torque"),
        g_MultiCard.MC_Update(1 << (kAxis4TorqueFollowerAxis - 1)));
    if (res != 0)
        return false;

    if (!SwitchAxisMode(kAxis4TorqueFollowerAxis, kModeCst, modeName))
        return false;
    if (!PrepareTorqueModeCommandSource(kAxis4TorqueFollowerAxis, modeName))
        return false;
    if (!EnableAxisCiA402(kAxis4TorqueFollowerAxis, modeName))
        return false;

    g_axisMode[kAxis4TorqueFollowerAxis] = AxisModeTorque;
    return true;
}

void ClearForceFeedbackCommands()
{
    SetTargetTorque(kAxis2TorqueLoopAxis, 0, QStringLiteral("Force feedback"), QStringLiteral("Clear axis2 torque 6071h"), false);
    SetTargetTorque(kAxis4TorqueFollowerAxis, 0, QStringLiteral("Force feedback"), QStringLiteral("Clear axis4 torque 6071h"), false);
    g_MultiCard.MC_Update(ForceFeedbackAxisMask());
}

void StopForceFeedbackAxes()
{
    ClearForceFeedbackCommands();
    DisableAxisDirect(kAxis2TorqueLoopAxis);
    DisableAxisDirect(kAxis4TorqueFollowerAxis);
}

long CalculateAxis2Torque(long targetPos, long currentPos, long axis2Velocity, double* integralPermille)
{
    long error = targetPos - currentPos;
    if (std::labs(error) <= kPositionDeadbandPulse)
    {
        *integralPermille = 0.0;
        return 0;
    }

    const double periodSecond = (double)kForceFeedbackPeriodUs / 1000000.0;
    double proportional = g_forceFeedbackMitKp.load() * (double)error;
    double derivative = -g_forceFeedbackMitKd.load() * (double)axis2Velocity;
    *integralPermille = ClampIntegral(*integralPermille +
        kAxis2KiPermillePerPulseSecond * (double)error * periodSecond);

    double torque = proportional + *integralPermille + derivative;
    double direction = Sign(torque);
    if (direction == 0.0)
        direction = Sign((double)error);
    torque += direction * kAxis2FrictionFeedForwardPermille;

    return ClampTorque(RoundToLong(torque));
}

long CalculateAxis4Torque(long grating1Position,
    long grating2Position,
    long axis2Velocity,
    long axis4Velocity,
    long axis2ActualTorque)
{
    long positionError = grating1Position - grating2Position;
    long velocityError = axis2Velocity - axis4Velocity;
    double torque =
        kAxis4Axis2TorqueFeedForwardScale * (double)axis2ActualTorque +
        g_forceFeedbackMitKp.load() * (double)positionError +
        g_forceFeedbackMitKd.load() * (double)velocityError;

    return ClampTorque(RoundToLong(torque));
}

}

long RoundToLong(double value)
{
    return (long)std::llround(value);
}

void StartForceFeedback(long targetGratingPos)
{
    StopAllGratingClosedLoop();
    if (g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("Force feedback start rejected: grating zero is running."));
        return;
    }
    if (g_gratingZeroThread.joinable())
        g_gratingZeroThread.join();

    if (g_bFollowRunning)
    {
        SetForceFeedbackVel(targetGratingPos);
        return;
    }

    if (g_forceFeedbackThread.joinable())
        g_forceFeedbackThread.join();

    g_forceFeedbackStopRequested = false;
    g_forceFeedbackCurrentVel = 0;
    g_forceFeedbackTargetVel = targetGratingPos;
    g_forceFeedbackTargetGrating2 = 0;
    g_bFollowRunning = true;
    g_forceFeedbackThread = std::thread([targetGratingPos]() {
        ForceFeedback(targetGratingPos);
        });

    logMessage(QStringLiteral("Force feedback started: grating1 target=%1 pulse, axis2 loop, axis4 torque follower.")
        .arg(targetGratingPos));
}

void SetForceFeedbackMitConfig(double kp, double kd)
{
    if (std::isfinite(kp))
        g_forceFeedbackMitKp = kp;
    if (std::isfinite(kd))
        g_forceFeedbackMitKd = kd;

    logMessage(QStringLiteral("Force feedback MIT config updated: Kp=%1, Kd=%2.")
        .arg(g_forceFeedbackMitKp.load(), 0, 'g', 8)
        .arg(g_forceFeedbackMitKd.load(), 0, 'g', 8));
}

void StartForceFeedback(long targetGrating1Pos, long)
{
    StartForceFeedback(targetGrating1Pos);
}

void SetForceFeedbackVel(long targetGratingPos)
{
    g_forceFeedbackTargetVel = targetGratingPos;
    g_forceFeedbackTargetGrating2 = 0;
    logMessage(QStringLiteral("Force feedback target updated: grating1=%1 pulse.")
        .arg(targetGratingPos));
}

void SetForceFeedbackTargets(long targetGrating1Pos, long)
{
    SetForceFeedbackVel(targetGrating1Pos);
}

void StopForceFeedback()
{
    g_forceFeedbackStopRequested = true;

    if (g_forceFeedbackThread.joinable())
        g_forceFeedbackThread.join();
    else
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
    }

    logMessage(QStringLiteral("Force feedback stopped."));
}

void ForceFeedback(long targetGratingPos)
{
    g_forceFeedbackTargetVel = targetGratingPos;
    g_forceFeedbackTargetGrating2 = 0;

    if (!IsCardOpened())
    {
        g_bFollowRunning = false;
        logMessage(QStringLiteral("Force feedback failed: card is not open."));
        return;
    }

    if (!PrepareAxis2TorqueLoop())
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
        logMessage(QStringLiteral("Force feedback failed: axis2 torque loop preparation failed."));
        return;
    }

    if (!PrepareAxis4TorqueFollower())
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
        logMessage(QStringLiteral("Force feedback failed: axis4 torque follower preparation failed."));
        return;
    }

    ClearForceFeedbackCommands();

    long grating1Pos = 0;
    long grating2Pos = 0;
    if (!ReadForceFeedbackGrating1(&grating1Pos))
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
        logMessage(QStringLiteral("Force feedback failed: grating1 read failed."));
        return;
    }
    if (!ReadForceFeedbackGrating2(&grating2Pos))
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
        logMessage(QStringLiteral("Force feedback failed: grating2 read failed."));
        return;
    }

    double integralPermille = 0.0;
    bool firstCommandLogged = false;
    auto nextTime = std::chrono::steady_clock::now();
    while (g_bFollowRunning)
    {
        auto execStart = std::chrono::steady_clock::now();
        nextTime += std::chrono::microseconds(kForceFeedbackPeriodUs);

        if (g_forceFeedbackStopRequested)
        {
            g_bFollowRunning = false;
            break;
        }

        long targetPos = g_forceFeedbackTargetVel.load();
        bool readGrating1Ok = ReadForceFeedbackGrating1(&grating1Pos);
        bool readGrating2Ok = ReadForceFeedbackGrating2(&grating2Pos);
        if (!readGrating1Ok || !readGrating2Ok)
        {
            ClearForceFeedbackCommands();
            integralPermille = 0.0;
            logMessage(QStringLiteral("Force feedback: grating read failed, commands cleared for this cycle."));
            std::this_thread::sleep_until(nextTime);
            continue;
        }

        long axis2Velocity = ReadVelocity(kAxis2TorqueLoopAxis);
        long axis2LoopTorqueCommand = CalculateAxis2Torque(targetPos, grating1Pos, axis2Velocity, &integralPermille);
        long axis2TorqueCommand = ToAxisTorqueCommand(kAxis2TorqueLoopAxis, axis2LoopTorqueCommand);
        SetTargetTorque(kAxis2TorqueLoopAxis,
            axis2TorqueCommand,
            QStringLiteral("Force feedback"),
            QStringLiteral("Set axis2 target torque 6071h"),
            false);

        long axis2ActualTorque = ToLoopTorqueFeedback(kAxis2TorqueLoopAxis, ReadTorque(kAxis2TorqueLoopAxis));
        long axis4Velocity = ReadVelocity(kAxis4TorqueFollowerAxis);
        long axis4LoopTorqueCommand = CalculateAxis4Torque(grating1Pos,
            grating2Pos,
            axis2Velocity,
            axis4Velocity,
            axis2ActualTorque);
        long axis4TorqueCommand = ToAxisTorqueCommand(kAxis4TorqueFollowerAxis, axis4LoopTorqueCommand);
        if (!firstCommandLogged)
        {
            firstCommandLogged = true;
            logMessage(QStringLiteral("Force feedback first command: target=%1, grating1=%2, grating2=%3, error=%4, axis2LoopTorque=%5, axis2Torque=%6, axis2ActualLoopTorque=%7, axis4LoopTorque=%8, axis4Torque=%9")
                .arg(targetPos)
                .arg(grating1Pos)
                .arg(grating2Pos)
                .arg(targetPos - grating1Pos)
                .arg(axis2LoopTorqueCommand)
                .arg(axis2TorqueCommand)
                .arg(axis2ActualTorque)
                .arg(axis4LoopTorqueCommand)
                .arg(axis4TorqueCommand));
        }
        SetTargetTorque(kAxis4TorqueFollowerAxis,
            axis4TorqueCommand,
            QStringLiteral("Force feedback"),
            QStringLiteral("Set axis4 target torque 6071h"),
            false);

        g_MultiCard.MC_Update(ForceFeedbackAxisMask());

        auto execEnd = std::chrono::steady_clock::now();
        long long execUs = std::chrono::duration_cast<std::chrono::microseconds>(execEnd - execStart).count();
        long long oldMaxUs = g_forceFeedbackExecMaxUs.load();
        while (execUs > oldMaxUs && !g_forceFeedbackExecMaxUs.compare_exchange_weak(oldMaxUs, execUs))
        {
        }

        std::this_thread::sleep_until(nextTime);
    }

    StopForceFeedbackAxes();
    g_forceFeedbackCurrentVel = 0;
    g_forceFeedbackStopRequested = false;
}

void ForceFeedback(long targetGrating1Pos, long)
{
    ForceFeedback(targetGrating1Pos);
}

long long ConsumeForceFeedbackExecMaxUs()
{
    return g_forceFeedbackExecMaxUs.exchange(0);
}
