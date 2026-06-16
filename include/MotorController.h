#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "MotorTypes.h"
#include <QTextBrowser>
#include <QString>

extern QTextBrowser* g_logWidget;

// 函数声明
void OpenCard();
void CloseCard();
void StartForceFeedback(long targetGratingPos);
void StartForceFeedback(long targetGrating1Pos, long targetGrating2Pos);
void SetForceFeedbackVel(long targetGratingPos);
void SetForceFeedbackTargets(long targetGrating1Pos, long targetGrating2Pos);
void StopForceFeedback();
void ForceFeedback(long targetGratingPos);
void ForceFeedback(long targetGrating1Pos, long targetGrating2Pos);
void SetForceFeedbackMitConfig(double kp, double kd);
void SetGratingClosedLoopConfig(double kp, double ki, double integralLimit, double kd, double frictionCompensation, long torqueLimit);
void StartGratingClosedLoop(int grating, long targetPos);
void StopGratingClosedLoop(int grating);
void StopAllGratingClosedLoop();
void PositionModeMove(int axis, long targetPos, long speed);
void VelocityModeMove(int axis, long speed);
void TorqueModeSet(int axis, long torque);
void DisableAxis(int axis);
void StartGratingZero(int grating);
bool ReadGratingSensorTriggered(int grating, bool* triggered);

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
