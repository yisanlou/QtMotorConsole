#include "MotorInternal.h"
#include <Windows.h>
#include <cstdint>

#pragma execution_character_set("utf-8")

static long ClampTorquePermille(long value)
{
    if (value > 1000)
        return 1000;
    if (value < -1000)
        return -1000;
    return value;
}

bool PrepareTorqueModeCommandSource(int axis, const QString& modeName)
{
    int ret = 0;
    long source = ReadSdoLong(axis, kIndexTorqueCommandSource, 0x00, &ret);
    if (ret == 0 && source == kTorqueCommandSourceBus6080)
        return true;

    int writeRet = g_MultiCard.MC_ECatSetSdoValue(
        axis,
        kIndexTorqueCommandSource,
        0x00,
        kTorqueCommandSourceBus6080,
        2);
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
    }

    return true;
}

int SetTargetTorque(int axis, long torque, const QString& modeName, const QString& stepName, bool verifyReadBack)
{
    long targetTorque = ClampTorquePermille(torque);
    if (targetTorque != torque)
    {
        logMessage(QStringLiteral("%1：目标力矩%2超出±1000范围，已限幅为%3")
            .arg(modeName)
            .arg(torque)
            .arg(targetTorque));
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

void PositionModeMove(int axis, long targetPos, long speed)
{
    const QString modeName = QStringLiteral("位置模式");

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();
    StopAllGratingClosedLoop();

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
    StopAllGratingClosedLoop();

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
    StopAllGratingClosedLoop();

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

    if ((axis == 2 || axis == 4) &&
        (g_bFollowRunning || g_forceFeedbackThread.joinable()))
    {
        StopForceFeedback();
    }
    if (axis == 2)
        StopGratingClosedLoop(1);
    else if (axis == 4)
        StopGratingClosedLoop(2);

    DisableAxisDirect(axis);
    logMessage(QStringLiteral("电机%1已失能。").arg(axis));
}



