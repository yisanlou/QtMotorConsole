#include "MotorInternal.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kForceFeedbackPeriodUs = 5000; // 5 ms, 200 Hz
constexpr double kPeriodSecond = (double)kForceFeedbackPeriodUs / 1000000.0;
constexpr int kMasterAxis = 2;
constexpr int kFollowerAxis = 4;
constexpr long kPositionDeadbandPulse = 500;
constexpr double kArrivalVelocityTolerancePulsePerSecond = 2000.0;
constexpr double kDefaultMasterKp = 0.0015;
constexpr double kDefaultMasterKd = 0.0010;
constexpr double kDefaultSyncKp = 0.0010;
constexpr double kDefaultSyncKd = 0.0015;
constexpr double kDefaultPlanVelocity = 100000.0;
constexpr double kDefaultPlanAcceleration = 200000.0;
constexpr long kDefaultTorqueLimit = 120;
constexpr long kDefaultTorqueModeMaxSpeedRpm = 3000;
constexpr long kTorqueSlewLimitPerCycle = 4;
constexpr double kMasterKi = 0.0010;
constexpr double kMasterIntegralLimit = 80.0;
constexpr double kIntegralMotionDecay = 0.95;
constexpr double kIntegralMotionVelocityPulsePerSecond = 5000.0;
// Each table has its own position loop. Axis 4 uses the measured axis-2
// torque as feed-forward and the relative grating motion as correction.
constexpr double kVelocityFilterCurrentWeight = 0.15;

std::atomic<double> g_masterKp(kDefaultMasterKp);
std::atomic<double> g_masterKd(kDefaultMasterKd);
std::atomic<double> g_syncKp(kDefaultSyncKp);
std::atomic<double> g_syncKd(kDefaultSyncKd);
std::atomic<double> g_planVelocity(kDefaultPlanVelocity);
std::atomic<double> g_planAcceleration(kDefaultPlanAcceleration);
std::atomic_long g_torqueLimit(kDefaultTorqueLimit);

std::atomic_long g_statusTarget(0);
std::atomic_long g_statusPlanned(0);
std::atomic_long g_statusGrating1(0);
std::atomic_long g_statusGrating2(0);
std::atomic_long g_statusSyncError(0);
std::atomic_long g_statusAxis2Torque(0);
std::atomic_long g_statusAxis2ActualTorque(0);
std::atomic_long g_statusAxis4Torque(0);
std::atomic_long g_statusAxis4ActualTorque(0);

struct TrajectoryState
{
    double position = 0.0;
    double velocity = 0.0;
};

double Sign(double value)
{
    if (value > 0.0)
        return 1.0;
    if (value < 0.0)
        return -1.0;
    return 0.0;
}

double Clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

long ClampTorque(double torque)
{
    long limit = std::labs(g_torqueLimit.load());
    if (limit <= 0)
        limit = kDefaultTorqueLimit;
    if (limit > 1000)
        limit = 1000;

    return RoundToLong(Clamp(torque, -(double)limit, (double)limit));
}

void UpdateAtomicMaximum(std::atomic_llong* maximum, long long value)
{
    long long oldMaximum = maximum->load();
    while (value > oldMaximum &&
        !maximum->compare_exchange_weak(oldMaximum, value))
    {
    }
}

long ApplyTorqueSlewLimit(long desiredTorque, long previousTorque)
{
    if (desiredTorque > previousTorque + kTorqueSlewLimitPerCycle)
        return previousTorque + kTorqueSlewLimitPerCycle;
    if (desiredTorque < previousTorque - kTorqueSlewLimitPerCycle)
        return previousTorque - kTorqueSlewLimitPerCycle;
    return desiredTorque;
}

double ClampIntegral(double value)
{
    return Clamp(value, -kMasterIntegralLimit, kMasterIntegralLimit);
}

double PositiveOrDefault(double value, double defaultValue)
{
    if (!std::isfinite(value))
        return defaultValue;
    value = std::fabs(value);
    return value > 0.0 ? value : defaultValue;
}

void AdvanceTrajectory(TrajectoryState* state, long target)
{
    const double maxVelocity = PositiveOrDefault(g_planVelocity.load(), kDefaultPlanVelocity);
    const double acceleration = PositiveOrDefault(g_planAcceleration.load(), kDefaultPlanAcceleration);
    const double remaining = (double)target - state->position;
    if (std::fabs(remaining) <= 0.5 && std::fabs(state->velocity) <= acceleration * kPeriodSecond)
    {
        state->position = (double)target;
        state->velocity = 0.0;
        return;
    }

    const double brakingVelocity = std::sqrt(2.0 * acceleration * std::fabs(remaining));
    const double desiredVelocity = Sign(remaining) * std::min(maxVelocity, brakingVelocity);
    const double maximumVelocityChange = acceleration * kPeriodSecond;
    state->velocity += Clamp(desiredVelocity - state->velocity,
        -maximumVelocityChange,
        maximumVelocityChange);

    const double nextPosition = state->position + state->velocity * kPeriodSecond;
    if ((remaining > 0.0 && nextPosition >= (double)target) ||
        (remaining < 0.0 && nextPosition <= (double)target))
    {
        state->position = (double)target;
        state->velocity = 0.0;
    }
    else
    {
        state->position = nextPosition;
    }
}

long ToAxisTorqueCommand(int axis, long controlTorque)
{
    return (long)CommonToAxisDirection(axis) * controlTorque;
}

int CopyAxis2TorqueConfigToAxis4(const QString& modeName)
{
    int result = 0;
    int ret = 0;

    const long maximumTorque = ReadSdoLong(kMasterAxis, 0x6072, 0x00, &ret);
    if (ret == 0 && maximumTorque > 0)
    {
        result = AddStepResult(result,
            modeName,
            QStringLiteral("Copy axis2 max torque 6072h to axis4"),
            g_MultiCard.MC_ECatSetSdoValue(kFollowerAxis, 0x6072, 0x00, maximumTorque, 2));
    }
    else if (ret != 0)
    {
        logMessage(QStringLiteral("%1：读取轴2 6072h失败，轴4保留当前配置，返回值=%2。")
            .arg(modeName)
            .arg(ret));
    }

    long maximumSpeedRpm = ReadSdoLong(kMasterAxis, 0x6080, 0x00, &ret);
    if (ret != 0 || maximumSpeedRpm <= 0)
    {
        maximumSpeedRpm = kDefaultTorqueModeMaxSpeedRpm;
        logMessage(QStringLiteral("%1：轴2的6080h读取无效，轴4采用手册默认最大速度%2 r/min。")
            .arg(modeName)
            .arg(maximumSpeedRpm));
    }
    result = AddStepResult(result,
        modeName,
        QStringLiteral("Copy torque-mode max speed 6080h to axis4"),
        g_MultiCard.MC_ECatSetSdoValue(
            kFollowerAxis, 0x6080, 0x00, maximumSpeedRpm, 4));

    return result;
}

bool PrepareTorqueAxis(int axis, bool copyMasterConfig)
{
    const QString modeName = axis == kMasterAxis
        ? QStringLiteral("双光栅同步：电机2")
        : QStringLiteral("双光栅同步：电机4");

    if (copyMasterConfig)
    {
        // These values are normally already stored in the drive. A transient
        // SDO write failure must not turn into a second axis-4 enable gate.
        CopyAxis2TorqueConfigToAxis4(modeName);
    }

    // First try the no-interruption transition that already works on axis 2.
    // If 6061 does not become CST, fall back below to the independent
    // torque-mode transition that is proven to work on the same axis.
    const int velocityRet =
        g_MultiCard.MC_ECatSetSdoValue(axis, 0x60FF, 0x00, 0, 4);
    const int torqueRet = SetTargetTorque(
        axis, 0, modeName, QStringLiteral("Clear target torque 6071h"), false);
    const int modeRet =
        g_MultiCard.MC_ECatSetSdoValue(axis, 0x6060, 0x00, kModeCst, 1);
    const int updateRet = g_MultiCard.MC_Update(1 << (axis - 1));
    if (velocityRet != 0 || torqueRet != 0 || modeRet != 0 || updateRet != 0)
    {
        logMessage(QStringLiteral("%1：直接切换CST返回：60FF=%2，6071=%3，6060=%4，Update=%5；继续启动。")
            .arg(modeName)
            .arg(velocityRet)
            .arg(torqueRet)
            .arg(modeRet)
            .arg(updateRet));
    }
    Sleep(30);

    bool usedModeFallback = false;
    if (ReadOperationMode(axis) != kModeCst)
    {
        logMessage(QStringLiteral("%1：在线切换CST未生效，改用独立力矩模式的切模/使能流程。")
            .arg(modeName));
        if (!SwitchAxisMode(axis, kModeCst, modeName))
            return false;
        usedModeFallback = true;
    }

    if (!PrepareTorqueModeCommandSource(axis, modeName))
        return false;

    if (!EnableAxisCiA402(axis, modeName))
        return false;

    const long displayMode = ReadOperationMode(axis);
    const short statusWord = ReadStatusWord(axis);
    if (displayMode != kModeCst || !IsOperationEnabled(statusWord))
    {
        logMessage(QStringLiteral("%1：CST准备失败，6061=0x%2，6041=0x%3。")
            .arg(modeName)
            .arg((unsigned long)displayMode, 2, 16, QChar('0'))
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
        return false;
    }

    logMessage(QStringLiteral("%1：CST准备完成，回退切模=%2，6061=0x%3，6041=0x%4。")
        .arg(modeName)
        .arg(usedModeFallback ? 1 : 0)
        .arg((unsigned long)displayMode, 2, 16, QChar('0'))
        .arg((unsigned short)statusWord, 4, 16, QChar('0')));

    if (axis == kFollowerAxis)
    {
        int speedRet = 0;
        const long maximumSpeedRpm =
            ReadSdoLong(axis, 0x6080, 0x00, &speedRet);
        int sourceRet = 0;
        const long torqueSource =
            ReadSdoLong(axis, kIndexTorqueCommandSource, 0x00, &sourceRet);
        const short statusWord = ReadStatusWord(axis);
        const long displayMode = ReadOperationMode(axis);
        if (speedRet != 0 ||
            maximumSpeedRpm <= 0 ||
            sourceRet != 0 ||
            torqueSource != kTorqueCommandSourceBus6080 ||
            displayMode != kModeCst ||
            !IsOperationEnabled(statusWord))
        {
            logMessage(QStringLiteral("%1：CST状态提示：6061=0x%2，6041=0x%3，PA25=%4(ret=%5)，6080=%6(ret=%7)；继续发送力矩指令。")
                .arg(modeName)
                .arg((unsigned long)displayMode, 2, 16, QChar('0'))
                .arg((unsigned short)statusWord, 4, 16, QChar('0'))
                .arg(torqueSource)
                .arg(sourceRet)
                .arg(maximumSpeedRpm)
                .arg(speedRet));
        }
        else
        {
            logMessage(QStringLiteral("%1：CST就绪，6041=0x%2，PA25=%3，6080=%4 r/min。")
                .arg(modeName)
                .arg((unsigned short)statusWord, 4, 16, QChar('0'))
                .arg(torqueSource)
                .arg(maximumSpeedRpm));
        }
    }

    g_axisMode[axis] = AxisModeTorque;
    return true;
}

void ClearForceFeedbackCommands()
{
    if (!CanAccessEtherCAT())
    {
        g_statusAxis2Torque = 0;
        g_statusAxis2ActualTorque = 0;
        g_statusAxis4Torque = 0;
        g_statusAxis4ActualTorque = 0;
        return;
    }

    const int followerRet = SetTargetTorque(kFollowerAxis,
        0,
        QStringLiteral("双光栅同步"),
        QStringLiteral("Clear axis4 torque 6071h"),
        false);
    const int followerUpdateRet =
        followerRet == 0
        ? g_MultiCard.MC_Update(1 << (kFollowerAxis - 1))
        : 0;
    const int masterRet = SetTargetTorque(kMasterAxis,
        0,
        QStringLiteral("双光栅同步"),
        QStringLiteral("Clear axis2 torque 6071h"),
        false);
    const int masterUpdateRet =
        masterRet == 0
        ? g_MultiCard.MC_Update(1 << (kMasterAxis - 1))
        : 0;

    g_statusAxis2Torque = 0;
    g_statusAxis2ActualTorque = 0;
    g_statusAxis4Torque = 0;
    g_statusAxis4ActualTorque = 0;
    if (followerRet != 0 ||
        followerUpdateRet != 0 ||
        masterRet != 0 ||
        masterUpdateRet != 0)
    {
        logMessage(QStringLiteral("双光栅同步零力矩返回：轴4写入/更新=%1/%2，轴2写入/更新=%3/%4。")
            .arg(followerRet)
            .arg(followerUpdateRet)
            .arg(masterRet)
            .arg(masterUpdateRet));
    }
}

void StopForceFeedbackAxes()
{
    ClearForceFeedbackCommands();
    // Keep both drives enabled. Explicit axis-disable and card-close paths
    // still perform the real disable when the operator requests it.
}

void UpdateStatus(long target,
    const TrajectoryState& trajectory,
    long grating1,
    long grating2,
    long syncError,
    long axis2Torque,
    long axis2ActualTorque,
    long axis4Torque,
    long axis4ActualTorque)
{
    g_statusTarget = target;
    g_statusPlanned = RoundToLong(trajectory.position);
    g_statusGrating1 = grating1;
    g_statusGrating2 = grating2;
    g_statusSyncError = syncError;
    g_statusAxis2Torque = axis2Torque;
    g_statusAxis2ActualTorque = axis2ActualTorque;
    g_statusAxis4Torque = axis4Torque;
    g_statusAxis4ActualTorque = axis4ActualTorque;
}

}

long RoundToLong(double value)
{
    return (long)std::llround(value);
}

void SetForceFeedbackMitConfig(double kp, double kd)
{
    if (std::isfinite(kp) && kp >= 0.0)
        g_masterKp = kp;
    if (std::isfinite(kd) && kd >= 0.0)
        g_masterKd = kd;
}

void SetForceFeedbackSyncConfig(double masterKp,
    double masterKd,
    double syncKp,
    double syncKd,
    double planVelocity,
    long torqueLimit)
{
    SetForceFeedbackMitConfig(masterKp, masterKd);
    if (std::isfinite(syncKp) && syncKp >= 0.0)
        g_syncKp = syncKp;
    if (std::isfinite(syncKd) && syncKd >= 0.0)
        g_syncKd = syncKd;

    g_planVelocity = PositiveOrDefault(planVelocity, kDefaultPlanVelocity);
    g_planAcceleration = std::max(1000.0, 2.0 * g_planVelocity.load());
    torqueLimit = std::labs(torqueLimit);
    g_torqueLimit = std::max(1L, std::min(1000L, torqueLimit));

    logMessage(QStringLiteral("双光栅同步参数：主Kp=%1，主Kd=%2，同步Kp=%3，同步Kd=%4，规划速度=%5 pulse/s，力矩限幅=%6。")
        .arg(g_masterKp.load(), 0, 'g', 8)
        .arg(g_masterKd.load(), 0, 'g', 8)
        .arg(g_syncKp.load(), 0, 'g', 8)
        .arg(g_syncKd.load(), 0, 'g', 8)
        .arg(g_planVelocity.load(), 0, 'f', 0)
        .arg(g_torqueLimit.load()));
}

ForceFeedbackStatus GetForceFeedbackStatus()
{
    ForceFeedbackStatus status;
    status.running = g_bFollowRunning.load();
    status.targetPosition = g_statusTarget.load();
    status.plannedPosition = g_statusPlanned.load();
    status.grating1Position = g_statusGrating1.load();
    status.grating2Position = g_statusGrating2.load();
    status.synchronizationError = g_statusSyncError.load();
    status.axis2TorqueCommand = g_statusAxis2Torque.load();
    status.axis2ActualTorque = g_statusAxis2ActualTorque.load();
    status.axis4TorqueCommand = g_statusAxis4Torque.load();
    status.axis4ActualTorque = g_statusAxis4ActualTorque.load();
    return status;
}

void StartForceFeedback(long targetGratingPos)
{
    StopAllGratingClosedLoop();
    if (g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("双光栅同步启动失败：光栅回零正在运行。"));
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
    g_forceFeedbackTargetVel = targetGratingPos;
    g_statusTarget = targetGratingPos;
    g_bFollowRunning = true;
    g_forceFeedbackThread = std::thread([targetGratingPos]() {
        ForceFeedback(targetGratingPos);
        });

    logMessage(QStringLiteral("双光栅柔性同步已启动：光栅1绝对目标=%1 pulse。").arg(targetGratingPos));
}

void StartForceFeedback(long targetGrating1Pos, long)
{
    StartForceFeedback(targetGrating1Pos);
}

void SetForceFeedbackVel(long targetGratingPos)
{
    g_forceFeedbackTargetVel = targetGratingPos;
    g_statusTarget = targetGratingPos;
    logMessage(QStringLiteral("双光栅同步目标更新：光栅1=%1 pulse。").arg(targetGratingPos));
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

    logMessage(QStringLiteral("双光栅柔性同步已停止。"));
}

void ForceFeedback(long targetGratingPos)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    g_forceFeedbackTargetVel = targetGratingPos;

    if (!IsCardOpened())
    {
        g_bFollowRunning = false;
        logMessage(QStringLiteral("双光栅同步启动失败：控制卡未打开。"));
        return;
    }

    if (!PrepareTorqueAxis(kMasterAxis, false))
    {
        g_bFollowRunning = false;
        ClearForceFeedbackCommands();
        logMessage(QStringLiteral("双光栅同步启动失败：电机2未进入CST。"));
        return;
    }
    if (!PrepareTorqueAxis(kFollowerAxis, true))
    {
        g_bFollowRunning = false;
        ClearForceFeedbackCommands();
        logMessage(QStringLiteral("双光栅同步启动失败：电机4未进入CST。"));
        return;
    }

    ClearForceFeedbackCommands();

    long grating1 = 0;
    long grating2 = 0;
    if (!ReadBothGratingPositions(&grating1, &grating2))
    {
        g_bFollowRunning = false;
        StopForceFeedbackAxes();
        logMessage(QStringLiteral("双光栅同步启动失败：光栅位置读取失败。"));
        return;
    }

    const long grating1Start = grating1;
    const long grating2Start = grating2;
    TrajectoryState trajectory;
    trajectory.position = (double)grating1;

    double integralTorque = 0.0;
    double filteredVelocity1 = 0.0;
    double filteredVelocity2 = 0.0;
    double previousMasterError = 0.0;
    long previousGrating1 = grating1;
    long previousGrating2 = grating2;
    long previousAxis2ControlTorque = 0;
    long previousAxis4TorqueCommand = 0;
    long lastSentAxis2TorqueCommand = 0;
    long lastSentAxis4TorqueCommand = 0;
    bool velocityFilterInitialized = false;
    bool firstCommandLogged = false;
    bool firstFollowerTorqueDiagnosed = false;
    bool gratingReadFailureLogged = false;
    bool torqueReadFailureLogged = false;
    bool axis4TorqueReadFailureLogged = false;
    bool targetTorqueReleased = false;
    bool arrivalZeroLogPending = false;
    long torqueReleaseTarget = targetGratingPos;
    auto nextTime = std::chrono::steady_clock::now();
    auto previousPositionTime = nextTime;

    while (g_bFollowRunning)
    {
        const auto execStart = std::chrono::steady_clock::now();
        nextTime += std::chrono::microseconds(kForceFeedbackPeriodUs);

        if (g_forceFeedbackStopRequested)
        {
            g_bFollowRunning = false;
            break;
        }

        const long target = g_forceFeedbackTargetVel.load();
        if (target != torqueReleaseTarget)
        {
            torqueReleaseTarget = target;
            targetTorqueReleased = false;
            arrivalZeroLogPending = false;
            integralTorque = 0.0;
            previousMasterError = 0.0;
            previousAxis2ControlTorque = 0;
            previousAxis4TorqueCommand = 0;
        }
        const auto gratingReadStart = std::chrono::steady_clock::now();
        const bool gratingReadOk = ReadBothGratingPositions(&grating1, &grating2);
        const auto gratingReadEnd = std::chrono::steady_clock::now();
        UpdateAtomicMaximum(&g_forceFeedbackGratingReadMaxUs,
            std::chrono::duration_cast<std::chrono::microseconds>(
                gratingReadEnd - gratingReadStart).count());
        if (!gratingReadOk)
        {
            ClearForceFeedbackCommands();
            integralTorque = 0.0;
            previousMasterError = 0.0;
            velocityFilterInitialized = false;
            previousAxis2ControlTorque = 0;
            previousAxis4TorqueCommand = 0;
            if (!gratingReadFailureLogged)
            {
                gratingReadFailureLogged = true;
                logMessage(QStringLiteral("双光栅同步：光栅读取失败，力矩已清零；恢复后自动继续。"));
            }
            const auto now = std::chrono::steady_clock::now();
            if (now > nextTime + std::chrono::microseconds(kForceFeedbackPeriodUs))
                nextTime = now;
            std::this_thread::sleep_until(nextTime);
            continue;
        }
        if (gratingReadFailureLogged)
        {
            gratingReadFailureLogged = false;
            logMessage(QStringLiteral("双光栅同步：光栅读取已恢复。"));
        }

        AdvanceTrajectory(&trajectory, target);

        // Position and damping now use the same grating-pulse coordinate.
        // This avoids the measured ~10.56:1 motor/grating pulse mismatch.
        const auto positionTime = std::chrono::steady_clock::now();
        if (!velocityFilterInitialized)
        {
            filteredVelocity1 = 0.0;
            filteredVelocity2 = 0.0;
            velocityFilterInitialized = true;
        }
        else
        {
            double samplePeriod =
                std::chrono::duration<double>(positionTime - previousPositionTime).count();
            samplePeriod = Clamp(samplePeriod, 0.001, 0.050);
            const double gratingVelocity1 =
                ((double)grating1 - (double)previousGrating1) / samplePeriod;
            const double gratingVelocity2 =
                ((double)grating2 - (double)previousGrating2) / samplePeriod;
            filteredVelocity1 +=
                kVelocityFilterCurrentWeight * (gratingVelocity1 - filteredVelocity1);
            filteredVelocity2 +=
                kVelocityFilterCurrentWeight * (gratingVelocity2 - filteredVelocity2);
        }
        previousGrating1 = grating1;
        previousGrating2 = grating2;
        previousPositionTime = positionTime;

        const double masterError = trajectory.position - (double)grating1;
        const bool trajectoryStopped =
            std::fabs(trajectory.position - (double)target) <= 0.5 &&
            std::fabs(trajectory.velocity) < 1.0;
        if (!targetTorqueReleased &&
            trajectoryStopped &&
            std::fabs(masterError) <= (double)kPositionDeadbandPulse &&
            std::fabs(filteredVelocity1) <= kArrivalVelocityTolerancePulsePerSecond)
        {
            targetTorqueReleased = true;
            arrivalZeroLogPending = true;
            integralTorque = 0.0;
            previousAxis2ControlTorque = 0;
            previousAxis4TorqueCommand = 0;
        }
        const bool crossedTarget =
            masterError * previousMasterError < 0.0;
        if (targetTorqueReleased ||
            crossedTarget ||
            (trajectoryStopped &&
                std::fabs(masterError) <= (double)kPositionDeadbandPulse))
        {
            integralTorque = 0.0;
        }
        else if (trajectoryStopped)
        {
            // Do not accumulate integral torque during acceleration/braking.
            // It is only used at rest to remove friction-related steady error.
            if (std::fabs(filteredVelocity1) >
                kIntegralMotionVelocityPulsePerSecond)
            {
                integralTorque *= kIntegralMotionDecay;
            }
            else
            {
                integralTorque = ClampIntegral(
                    integralTorque + kMasterKi * masterError * kPeriodSecond);
            }
        }
        else
        {
            integralTorque = 0.0;
        }
        previousMasterError = masterError;

        const double masterTorque =
            g_masterKp.load() * masterError +
            g_masterKd.load() * (trajectory.velocity - filteredVelocity1) +
            integralTorque;

        const long displacement1 = grating1 - grating1Start;
        const long displacement2 = grating2 - grating2Start;
        const long syncError = displacement1 - displacement2;

        long axis2ControlTorque = 0;
        if (!targetTorqueReleased)
        {
            const long desiredAxis2ControlTorque = ClampTorque(masterTorque);
            axis2ControlTorque =
                ApplyTorqueSlewLimit(desiredAxis2ControlTorque, previousAxis2ControlTorque);
        }
        previousAxis2ControlTorque = axis2ControlTorque;
        const long axis2TorqueCommand = ToAxisTorqueCommand(kMasterAxis, axis2ControlTorque);

        // Axis 4 follows axis 2's measured torque, while the relative
        // displacement/velocity term keeps the independent tables aligned.
        const auto actualTorqueReadStart = std::chrono::steady_clock::now();
        int actualTorqueReadRet = 0;
        int axis4ActualTorqueReadRet = 0;
        const long axis2ActualTorque = (int16_t)ReadSdoLong(
            kMasterAxis, 0x6077, 0x00, &actualTorqueReadRet);
        const long axis4ActualTorque = (int16_t)ReadSdoLong(
            kFollowerAxis, 0x6077, 0x00, &axis4ActualTorqueReadRet);
        const auto actualTorqueReadEnd = std::chrono::steady_clock::now();
        UpdateAtomicMaximum(&g_forceFeedbackActualTorqueReadMaxUs,
            std::chrono::duration_cast<std::chrono::microseconds>(
                actualTorqueReadEnd - actualTorqueReadStart).count());

        long axis4TorqueCommand = 0;
        const double syncCorrectionCommon =
            g_syncKp.load() * (double)syncError +
            g_syncKd.load() * (filteredVelocity1 - filteredVelocity2);
        if (actualTorqueReadRet == 0)
        {
            if (!targetTorqueReleased)
            {
                const long desiredAxis4TorqueCommand = ClampTorque(
                    -(double)axis2ActualTorque +
                    (double)CommonToAxisDirection(kFollowerAxis) *
                    syncCorrectionCommon);
                axis4TorqueCommand = ApplyTorqueSlewLimit(
                    desiredAxis4TorqueCommand,
                    previousAxis4TorqueCommand);
            }
            if (torqueReadFailureLogged)
            {
                torqueReadFailureLogged = false;
                logMessage(QStringLiteral("双光栅同步：电机2实际力矩读取已恢复。"));
            }
        }
        else if (!torqueReadFailureLogged)
        {
            torqueReadFailureLogged = true;
            logMessage(QStringLiteral("双光栅同步：读取电机2实际力矩6077h失败，返回值=%1；电机4力矩已清零。")
                .arg(actualTorqueReadRet));
        }
        previousAxis4TorqueCommand = axis4TorqueCommand;
        if (axis4ActualTorqueReadRet == 0)
        {
            if (axis4TorqueReadFailureLogged)
            {
                axis4TorqueReadFailureLogged = false;
                logMessage(QStringLiteral("双光栅同步：电机4实际力矩读取已恢复。"));
            }
        }
        else if (!axis4TorqueReadFailureLogged)
        {
            axis4TorqueReadFailureLogged = true;
            logMessage(QStringLiteral("双光栅同步：读取电机4实际力矩6077h失败，返回值=%1。")
                .arg(axis4ActualTorqueReadRet));
        }

        const auto motorSampleStart = std::chrono::steady_clock::now();
        CaptureForceFeedbackSample(
            grating1,
            grating2,
            axis2ActualTorque,
            axis4ActualTorque,
            axis2TorqueCommand,
            axis4TorqueCommand);
        const auto motorSampleEnd = std::chrono::steady_clock::now();
        UpdateAtomicMaximum(&g_forceFeedbackMotorSampleMaxUs,
            std::chrono::duration_cast<std::chrono::microseconds>(
                motorSampleEnd - motorSampleStart).count());

        const auto torqueWriteStart = std::chrono::steady_clock::now();
        int axis2TorqueWriteRet = 0;
        int axis2TorqueUpdateRet = 0;
        if (axis2TorqueCommand != lastSentAxis2TorqueCommand)
        {
            axis2TorqueWriteRet = SetTargetTorque(kMasterAxis,
                axis2TorqueCommand,
                QStringLiteral("双光栅同步"),
                QStringLiteral("Set axis2 target torque 6071h"),
                false);
            if (axis2TorqueWriteRet == 0)
            {
                axis2TorqueUpdateRet =
                    g_MultiCard.MC_Update(1 << (kMasterAxis - 1));
                if (axis2TorqueUpdateRet == 0)
                    lastSentAxis2TorqueCommand = axis2TorqueCommand;
            }
        }

        int axis4TorqueWriteRet = 0;
        int axis4TorqueUpdateRet = 0;
        if (axis4TorqueCommand != lastSentAxis4TorqueCommand)
        {
            axis4TorqueWriteRet = SetTargetTorque(kFollowerAxis,
                axis4TorqueCommand,
                QStringLiteral("双光栅同步"),
                QStringLiteral("Set axis4 target torque 6071h"),
                false);
            if (axis4TorqueWriteRet == 0)
            {
                axis4TorqueUpdateRet =
                    g_MultiCard.MC_Update(1 << (kFollowerAxis - 1));
                if (axis4TorqueUpdateRet == 0)
                    lastSentAxis4TorqueCommand = axis4TorqueCommand;
            }
        }
        const auto torqueWriteEnd = std::chrono::steady_clock::now();
        UpdateAtomicMaximum(&g_forceFeedbackTorqueWriteMaxUs,
            std::chrono::duration_cast<std::chrono::microseconds>(
                torqueWriteEnd - torqueWriteStart).count());

        const auto updateEnd = std::chrono::steady_clock::now();
        UpdateAtomicMaximum(&g_forceFeedbackUpdateMaxUs,
            std::chrono::duration_cast<std::chrono::microseconds>(
                updateEnd - torqueWriteStart).count());
        if (arrivalZeroLogPending &&
            axis2TorqueWriteRet == 0 &&
            axis4TorqueWriteRet == 0 &&
            axis2TorqueUpdateRet == 0 &&
            axis4TorqueUpdateRet == 0)
        {
            arrivalZeroLogPending = false;
            logMessage(QStringLiteral("双光栅同步：光栅1已到位，电机2和电机4目标力矩已清零。"));
        }

        if (!firstFollowerTorqueDiagnosed && axis4TorqueCommand != 0)
        {
            firstFollowerTorqueDiagnosed = true;
            int targetRet = 0;
            int actualRet = 0;
            const long targetReadBack = (int16_t)ReadSdoLong(
                kFollowerAxis, 0x6071, 0x00, &targetRet);
            const long actualTorque4 = (int16_t)ReadSdoLong(
                kFollowerAxis, 0x6077, 0x00, &actualRet);
            const short statusWord4 = ReadStatusWord(kFollowerAxis);
            const long displayMode4 = ReadOperationMode(kFollowerAxis);
            logMessage(QStringLiteral("电机4首个非零力矩验证：指令=%1，6071读回=%2(ret=%3)，6077实际=%4(ret=%5)，6061=0x%6，6041=0x%7，OperationEnabled=%8，CST目标有效(Bit12)=%9。")
                .arg(axis4TorqueCommand)
                .arg(targetReadBack)
                .arg(targetRet)
                .arg(actualTorque4)
                .arg(actualRet)
                .arg((unsigned long)displayMode4, 2, 16, QChar('0'))
                .arg((unsigned short)statusWord4, 4, 16, QChar('0'))
                .arg(IsOperationEnabled(statusWord4) ? 1 : 0)
                .arg(displayMode4 == kModeCst &&
                    (statusWord4 & 0x1000) != 0 ? 1 : 0));
        }

        UpdateStatus(target,
            trajectory,
            grating1,
            grating2,
            syncError,
            axis2TorqueCommand,
            axis2ActualTorque,
            axis4TorqueCommand,
            axis4ActualTorque);

        if (!firstCommandLogged)
        {
            firstCommandLogged = true;
            logMessage(QStringLiteral("主从力矩跟随首周期：起点q1=%1，q2=%2，目标=%3，规划=%4，位置差=%5，轴2实际力矩=%6，轴4反向指令=%7。")
                .arg(grating1Start)
                .arg(grating2Start)
                .arg(target)
                .arg(RoundToLong(trajectory.position))
                .arg(syncError)
                .arg(axis2ActualTorque)
                .arg(axis4TorqueCommand));
        }

        const auto execEnd = std::chrono::steady_clock::now();
        const long long execUs =
            std::chrono::duration_cast<std::chrono::microseconds>(execEnd - execStart).count();
        UpdateAtomicMaximum(&g_forceFeedbackExecMaxUs, execUs);

        const auto now = std::chrono::steady_clock::now();
        if (now > nextTime + std::chrono::microseconds(kForceFeedbackPeriodUs))
            nextTime = now;
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

ForceFeedbackTiming ConsumeForceFeedbackTiming()
{
    ForceFeedbackTiming timing;
    timing.totalMaxUs = g_forceFeedbackExecMaxUs.exchange(0);
    timing.gratingReadMaxUs = g_forceFeedbackGratingReadMaxUs.exchange(0);
    timing.actualTorqueReadMaxUs = g_forceFeedbackActualTorqueReadMaxUs.exchange(0);
    timing.motorSampleMaxUs = g_forceFeedbackMotorSampleMaxUs.exchange(0);
    timing.torqueWriteMaxUs = g_forceFeedbackTorqueWriteMaxUs.exchange(0);
    timing.updateMaxUs = g_forceFeedbackUpdateMaxUs.exchange(0);
    return timing;
}
