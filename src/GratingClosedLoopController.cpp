#include "MotorInternal.h"
#include <chrono>
#include <cmath>
#include <thread>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kLoopPeriodUs = 5000;
constexpr long kPositionDeadbandPulse = 5;
constexpr double kDefaultAxisPulsePerGratingPulse = 1.0;
constexpr double kDefaultPlanVelocityPulsePerSecond = 50000.0;
constexpr double kDefaultPlanAccelerationPulsePerSecond2 = 200000.0;

// ReadGratingPosition already converts both gratings into the common coordinate.
constexpr int kGrating1DisplayToControlDirection = 1;
constexpr int kGrating2DisplayToControlDirection = 1;

std::atomic<double> g_axisPulsePerGratingPulse(kDefaultAxisPulsePerGratingPulse);
std::atomic<double> g_planVelocityPulsePerSecond(kDefaultPlanVelocityPulsePerSecond);
std::atomic<double> g_planAccelerationPulsePerSecond2(kDefaultPlanAccelerationPulsePerSecond2);

struct TrapPlan
{
    long startPos = 0;
    long targetPos = 0;
    double distance = 0.0;
    double direction = 1.0;
    double maxVelocity = kDefaultPlanVelocityPulsePerSecond;
    double acceleration = kDefaultPlanAccelerationPulsePerSecond2;
    double accelTime = 0.0;
    double cruiseTime = 0.0;
    double totalTime = 0.0;
    bool doneAtStart = false;
};

int AxisForGrating(int grating)
{
    return grating == 2 ? 4 : 2;
}

int DisplayToControlDirectionForGrating(int grating)
{
    return grating == 2 ? kGrating2DisplayToControlDirection : kGrating1DisplayToControlDirection;
}

int ControlToAxisDirectionForGrating(int grating)
{
    return CommonToAxisDirection(AxisForGrating(grating));
}

long ToControlPosition(int grating, long displayPosition)
{
    return (long)DisplayToControlDirectionForGrating(grating) * displayPosition;
}

long ToDisplayPosition(int grating, long controlPosition)
{
    return (long)DisplayToControlDirectionForGrating(grating) * controlPosition;
}

double PositiveOrDefault(double value, double defaultValue)
{
    if (value < 0.0)
        value = -value;
    if (value <= 0.0)
        return defaultValue;
    return value;
}

TrapPlan CreateTrapPlan(long startPos, long targetPos)
{
    TrapPlan plan;
    plan.startPos = startPos;
    plan.targetPos = targetPos;
    plan.distance = std::fabs((double)targetPos - (double)startPos);
    plan.direction = targetPos >= startPos ? 1.0 : -1.0;
    plan.maxVelocity = PositiveOrDefault(g_planVelocityPulsePerSecond.load(), kDefaultPlanVelocityPulsePerSecond);
    plan.acceleration = PositiveOrDefault(g_planAccelerationPulsePerSecond2.load(), kDefaultPlanAccelerationPulsePerSecond2);

    if (plan.distance <= (double)kPositionDeadbandPulse)
    {
        plan.doneAtStart = true;
        return plan;
    }

    double nominalAccelTime = plan.maxVelocity / plan.acceleration;
    double accelDistance = 0.5 * plan.acceleration * nominalAccelTime * nominalAccelTime;
    if (plan.distance <= 2.0 * accelDistance)
    {
        plan.accelTime = std::sqrt(plan.distance / plan.acceleration);
        plan.cruiseTime = 0.0;
        plan.maxVelocity = plan.acceleration * plan.accelTime;
    }
    else
    {
        plan.accelTime = nominalAccelTime;
        plan.cruiseTime = (plan.distance - 2.0 * accelDistance) / plan.maxVelocity;
    }

    plan.totalTime = 2.0 * plan.accelTime + plan.cruiseTime;
    return plan;
}

long PlannedLoopPositionAt(const TrapPlan& plan, double elapsedSecond)
{
    if (plan.doneAtStart || elapsedSecond >= plan.totalTime)
        return plan.targetPos;

    double moved = 0.0;
    if (elapsedSecond <= plan.accelTime)
    {
        moved = 0.5 * plan.acceleration * elapsedSecond * elapsedSecond;
    }
    else if (elapsedSecond <= plan.accelTime + plan.cruiseTime)
    {
        double cruiseElapsed = elapsedSecond - plan.accelTime;
        double accelDistance = 0.5 * plan.acceleration * plan.accelTime * plan.accelTime;
        moved = accelDistance + plan.maxVelocity * cruiseElapsed;
    }
    else
    {
        double decelElapsed = elapsedSecond - plan.accelTime - plan.cruiseTime;
        double accelDistance = 0.5 * plan.acceleration * plan.accelTime * plan.accelTime;
        double cruiseDistance = plan.maxVelocity * plan.cruiseTime;
        moved = accelDistance + cruiseDistance +
            plan.maxVelocity * decelElapsed -
            0.5 * plan.acceleration * decelElapsed * decelElapsed;
    }

    if (moved > plan.distance)
        moved = plan.distance;
    return plan.startPos + RoundToLong(plan.direction * moved);
}

long ToAxisTargetPosition(int grating, long axisStartPos, long loopStartPos, long plannedLoopPos)
{
    double loopDelta = (double)plannedLoopPos - (double)loopStartPos;
    double axisDelta = (double)ControlToAxisDirectionForGrating(grating) *
        g_axisPulsePerGratingPulse.load() * loopDelta;
    return axisStartPos + RoundToLong(axisDelta);
}

long AxisTargetFromControlError(int controlledGrating, long axisActualPos, long controlError)
{
    double axisCorrection = (double)ControlToAxisDirectionForGrating(controlledGrating) *
        g_axisPulsePerGratingPulse.load() * (double)controlError;
    return axisActualPos + RoundToLong(axisCorrection);
}

bool PrepareLoopAxis(int grating)
{
    const int axis = AxisForGrating(grating);
    const QString modeName = QStringLiteral("Grating position loop");

    int res = 0;
    res = AddStepResult(res, modeName, QStringLiteral("Clear target torque 6071h"),
        SetTargetTorque(axis, 0, modeName, QStringLiteral("Clear target torque 6071h"), false));
    res = AddStepResult(res, modeName, QStringLiteral("Update zero torque"),
        g_MultiCard.MC_Update(1 << (axis - 1)));
    if (res != 0)
        return false;

    if (!SwitchAxisMode(axis, kModeCsp, modeName))
        return false;
    if (!EnableAxisCiA402(axis, modeName))
        return false;

    res = AddStepResult(res, modeName, QStringLiteral("Switch trap planner"), g_MultiCard.MC_PrfTrap(axis));
    res = AddStepResult(res, modeName, QStringLiteral("Hold current target position"), g_MultiCard.MC_SetPos(axis, ReadPosition(axis)));
    res = AddStepResult(res, modeName, QStringLiteral("Update hold position"), g_MultiCard.MC_Update(1 << (axis - 1)));
    if (res != 0)
        return false;

    g_axisMode[axis] = AxisModePosition;
    return true;
}

void HoldLoopAxis(int grating)
{
    const int axis = AxisForGrating(grating);
    g_MultiCard.MC_SetPos(axis, ReadPosition(axis));
    g_MultiCard.MC_Update(1 << (axis - 1));
}

long CurrentUiGratingPosition(int grating)
{
    return grating == 2 ? g_uiGrating2.load() : g_uiGrating1.load();
}

void Grating2FollowGrating1Worker()
{
    constexpr int kControlledGrating = 2;
    constexpr int kReferenceGrating = 1;
    const int axis = AxisForGrating(kControlledGrating);

    bool firstCommandLogged = false;
    auto nextTime = std::chrono::steady_clock::now();
    while (g_gratingClosedLoopRunning[kControlledGrating])
    {
        nextTime += std::chrono::microseconds(kLoopPeriodUs);

        if (!g_uiGratingSampleValid.load())
        {
            HoldLoopAxis(kControlledGrating);
            logMessage(QStringLiteral("Grating 2 input from grating 1: UI grating sample is not ready, holding axis 4."));
            std::this_thread::sleep_until(nextTime);
            continue;
        }

        long grating1CurrentUiPos = CurrentUiGratingPosition(kReferenceGrating);
        long grating2CurrentUiPos = CurrentUiGratingPosition(kControlledGrating);
        long grating2InputControlPos = ToControlPosition(kControlledGrating, grating1CurrentUiPos);
        long grating2ActualControlPos = ToControlPosition(kControlledGrating, grating2CurrentUiPos);
        long grating2ControlError = grating2InputControlPos - grating2ActualControlPos;
        long axisActualPos = ReadPosition(axis);
        long axisTargetPos = AxisTargetFromControlError(kControlledGrating, axisActualPos, grating2ControlError);

        if (!firstCommandLogged)
        {
            firstCommandLogged = true;
            logMessage(QStringLiteral("Grating 2 input from grating 1: grating1Input=%1, grating2Actual=%2, inputCtrl=%3, actualCtrl=%4, errCtrl=%5, axis4Actual=%6, axis4Target=%7, ratio=%8")
                .arg(grating1CurrentUiPos)
                .arg(grating2CurrentUiPos)
                .arg(grating2InputControlPos)
                .arg(grating2ActualControlPos)
                .arg(grating2ControlError)
                .arg(axisActualPos)
                .arg(axisTargetPos)
                .arg(g_axisPulsePerGratingPulse.load(), 0, 'f', 6)
            );
        }

        g_MultiCard.MC_SetPos(axis, axisTargetPos);
        g_MultiCard.MC_Update(1 << (axis - 1));
        std::this_thread::sleep_until(nextTime);
    }

    HoldLoopAxis(kControlledGrating);
    DisableAxisDirect(axis);
}

void GratingClosedLoopWorker(int grating, long targetPos)
{
    const int axis = AxisForGrating(grating);
    if (!IsCardOpened())
    {
        logMessage(QStringLiteral("Grating %1 position loop failed: card is not open.").arg(grating));
        g_gratingClosedLoopRunning[grating] = false;
        return;
    }

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();
    if (g_gratingZeroRunning)
    {
        logMessage(QStringLiteral("Grating %1 position loop rejected: grating zero is running.").arg(grating));
        g_gratingClosedLoopRunning[grating] = false;
        return;
    }
    if (g_gratingZeroThread.joinable())
        g_gratingZeroThread.join();

    if (!PrepareLoopAxis(grating))
    {
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 position loop failed: axis %2 preparation failed.").arg(grating).arg(axis));
        return;
    }

    if (!g_uiGratingSampleValid.load())
    {
        HoldLoopAxis(grating);
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 position loop failed: UI grating sample is not ready.").arg(grating));
        return;
    }

    if (grating == 2)
    {
        Q_UNUSED(targetPos);
        Grating2FollowGrating1Worker();
        return;
    }

    long startUiPosition = CurrentUiGratingPosition(grating);
    long loopStartPos = ToControlPosition(grating, startUiPosition);
    long loopTargetPos = ToControlPosition(grating, targetPos);
    long axisStartPos = ReadPosition(axis);
    TrapPlan plan = CreateTrapPlan(loopStartPos, loopTargetPos);
    bool firstCommandLogged = false;
    bool completed = false;
    auto startTime = std::chrono::steady_clock::now();
    auto nextTime = startTime;
    while (g_gratingClosedLoopRunning[grating])
    {
        nextTime += std::chrono::microseconds(kLoopPeriodUs);
        double elapsedSecond = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
        long plannedLoopPos = PlannedLoopPositionAt(plan, elapsedSecond);
        long axisTargetPos = ToAxisTargetPosition(grating, axisStartPos, loopStartPos, plannedLoopPos);

        if (!firstCommandLogged)
        {
            firstCommandLogged = true;
            logMessage(QStringLiteral("Grating %1 CSP trap plan: uiStart=%2, uiTarget=%3, loopStart=%4, loopTarget=%5, distance=%6, vmax=%7, acc=%8, total=%9s, axis%10Start=%11, ratio=%12")
                .arg(grating)
                .arg(startUiPosition)
                .arg(targetPos)
                .arg(loopStartPos)
                .arg(loopTargetPos)
                .arg(plan.distance, 0, 'f', 0)
                .arg(plan.maxVelocity, 0, 'f', 0)
                .arg(plan.acceleration, 0, 'f', 0)
                .arg(plan.totalTime, 0, 'f', 3)
                .arg(axis)
                .arg(axisStartPos)
                .arg(g_axisPulsePerGratingPulse.load(), 0, 'f', 6));
        }

        g_MultiCard.MC_SetPos(axis, axisTargetPos);
        g_MultiCard.MC_Update(1 << (axis - 1));

        if (plan.doneAtStart || elapsedSecond >= plan.totalTime)
        {
            completed = true;
            break;
        }

        std::this_thread::sleep_until(nextTime);
    }

    if (completed)
    {
        long finalAxisTarget = ToAxisTargetPosition(grating, axisStartPos, loopStartPos, plan.targetPos);
        g_MultiCard.MC_SetPos(axis, finalAxisTarget);
        g_MultiCard.MC_Update(1 << (axis - 1));
        g_gratingClosedLoopRunning[grating] = false;
        logMessage(QStringLiteral("Grating %1 CSP trap plan complete: target=%2, axis%3Target=%4")
            .arg(grating)
            .arg(ToDisplayPosition(grating, plan.targetPos))
            .arg(axis)
            .arg(finalAxisTarget));
        return;
    }

    HoldLoopAxis(grating);
    DisableAxisDirect(axis);
}

}

void SetGratingClosedLoopConfig(double kp, double ki, double integralLimit, double kd, double frictionCompensation, long torqueLimit)
{
    g_axisPulsePerGratingPulse = kp;
    g_planVelocityPulsePerSecond = ki;
    g_planAccelerationPulsePerSecond2 = integralLimit;
    Q_UNUSED(kd);
    Q_UNUSED(frictionCompensation);
    Q_UNUSED(torqueLimit);
}

void StartGratingClosedLoop(int grating, long targetPos)
{
    if (grating != 1 && grating != 2)
        return;

    if (g_gratingClosedLoopRunning[grating])
    {
        logMessage(QStringLiteral("Grating %1 position loop target updated requires restart; stop first.").arg(grating));
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
