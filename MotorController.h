#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "MultiCardCPP.h"
#include <Windows.h>  // Sleep
#include <atomic>
#include <QTextBrowser>
#include <QString>
#include <QTextStream>

// 全局变量
extern MultiCard g_MultiCard;
extern std::atomic_bool g_bFollowRunning;
extern std::atomic_bool g_cardOpened;
extern QTextBrowser* g_logWidget;

struct MotorSample
{
    long pos1 = 0;
    long pos2 = 0;
    long pos3 = 0;
    long pos4 = 0;
    long vel1 = 0;
    long vel2 = 0;
    long vel3 = 0;
    long vel4 = 0;
    long torque1 = 0;
    long torque2 = 0;
    long torque3 = 0;
    long torque4 = 0;
    long grating1 = 0;
    long grating2 = 0;
};

// 函数声明
void OpenCard();
void CloseCard();
void StartForceFeedback(long vel);
void SetForceFeedbackVel(long vel);
void StopForceFeedback();
void ForceFeedback(long vel);
void PositionModeMove(int axis, long targetPos, long speed);
void VelocityModeMove(int axis, long speed);
void TorqueModeSet(int axis, long torque);
void DisableAxis(int axis);
void StartGratingZero(int grating);

long ReadPosition(int axis);
long ReadVelocity(int axis);
long ReadTorque(int axis);
MotorSample ReadFastSample();

// 日志输出函数
void logMessage(const QString& msg);

// 配置伺服参数
bool ConfigServo();
bool IsCardOpened();
long long ConsumeForceFeedbackExecMaxUs();
long ReadOperationMode(int axis);

#endif // MOTORCONTROLLER_H
