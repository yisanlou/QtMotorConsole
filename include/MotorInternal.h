#ifndef MOTORINTERNAL_H
#define MOTORINTERNAL_H

#include "MotorController.h"
#include "MultiCardCPP.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <QString>

enum AxisWorkMode
{
    AxisModeUnknown = 0,
    AxisModePosition,
    AxisModeVelocity,
    AxisModeTorque
};

constexpr int kMaxAxisCount = 4;
constexpr unsigned short kModeCsp = 0x08;
constexpr unsigned short kModeCsv = 0x09;
constexpr unsigned short kModeCst = 0x0A;
constexpr int kIndexTorqueCommandSource = 0x2019; // PA25
constexpr int kTorqueCommandSourceBus6080 = 3;
constexpr unsigned short kControlShutdown = 0x0006;
constexpr unsigned short kControlSwitchOn = 0x0007;
constexpr unsigned short kControlEnableOperation = 0x000F;
constexpr unsigned short kControlFaultReset = 0x0080;

// Common motion coordinate:
// - motor 2 / grating 1 physical forward is positive;
// - the motor drive directions are opposite;
// - homing data confirms both grating raw-minus-offset values require a
//   negative sign to increase in the common physical direction.
// A zeroed grating position is:
//     rawToCommonDirection * (rawEncoderValue - homingOffset)
constexpr int kGrating1RawToCommonDirection = -1;
constexpr int kGrating2RawToCommonDirection = -1;
constexpr int kAxis2CommonToDriveDirection = -1;
constexpr int kAxis4CommonToDriveDirection = 1;

constexpr int GratingRawToCommonDirection(int grating)
{
    return grating == 2
        ? kGrating2RawToCommonDirection
        : kGrating1RawToCommonDirection;
}

constexpr int CommonToAxisDirection(int axis)
{
    return axis == 4
        ? kAxis4CommonToDriveDirection
        : kAxis2CommonToDriveDirection;
}

extern MultiCard g_MultiCard;
extern std::atomic_bool g_bFollowRunning;
extern std::atomic_bool g_cardOpened;
extern std::atomic_bool g_forceFeedbackStopRequested;
extern std::atomic_long g_forceFeedbackTargetVel;
extern std::atomic_long g_forceFeedbackTargetGrating2;
extern std::atomic_long g_forceFeedbackCurrentVel;
extern std::atomic_llong g_forceFeedbackExecMaxUs;
extern std::atomic_llong g_forceFeedbackGratingReadMaxUs;
extern std::atomic_llong g_forceFeedbackActualTorqueReadMaxUs;
extern std::atomic_llong g_forceFeedbackMotorSampleMaxUs;
extern std::atomic_llong g_forceFeedbackTorqueWriteMaxUs;
extern std::atomic_llong g_forceFeedbackUpdateMaxUs;
extern std::thread g_forceFeedbackThread;
extern std::atomic_bool g_gratingZeroRunning;
extern std::thread g_gratingZeroThread;
extern std::atomic_bool g_gratingClosedLoopRunning[3];
extern std::thread g_gratingClosedLoopThread[3];
extern AxisWorkMode g_axisMode[kMaxAxisCount + 1];
extern std::atomic_int g_activeAxisCount;
extern std::atomic_bool g_cardClosing;
extern std::atomic_long g_sampleTorqueCache[kMaxAxisCount];
extern int g_nextSampleTorqueAxis;
extern bool g_grating1ReadErrorLogged;
extern bool g_grating2ReadErrorLogged;
extern int g_grating2EncoderIndex;
extern long g_grating2LastValue;
extern int g_grating2UnchangedCount;
extern std::atomic_long g_gratingOffset1;
extern std::atomic_long g_gratingOffset2;
extern std::atomic_long g_uiGrating1;
extern std::atomic_long g_uiGrating2;
extern std::atomic_bool g_uiGratingSampleValid;
extern std::mutex g_forceFeedbackSampleMutex;
extern MotorSample g_forceFeedbackSample;
extern bool g_forceFeedbackSampleValid;

void ResetFastSampleCache();
bool IsValidAxis(int axis);
bool CanAccessEtherCAT();
long ReadSdoLong(int axis, int index, int subIndex, int* ret = nullptr);
int SetSdoWithLog(int axis, int index, int subIndex, long value, short len, const QString& name);
short ReadStatusWord(int axis, int* ret = nullptr);
bool WaitAxisOnline(int axis, long* supportedModes, long* displayMode, short* statusWord);
int WriteControlWord(int axis, unsigned short value, const QString& stepName);
long RoundToLong(double value);
bool IsOperationEnabled(short statusWord);
bool EnableAxisCiA402(int axis, const QString& modeName);
bool SwitchAxisMode(int axis, unsigned short mode, const QString& modeName);
void PrepareAxisForDisable(int axis);
void DisableAxisDirect(int axis);
void DisableAllAxesForClose();
int AddStepResult(int total, const QString& modeName, const QString& stepName, int ret);
bool PrepareTorqueModeCommandSource(int axis, const QString& modeName);
int SetTargetTorque(int axis, long torque, const QString& modeName, const QString& stepName, bool verifyReadBack = true);
bool ReadGratingEncoder(int encoderIndex, long* value, int* ret = nullptr);
bool ReadGratingPosition(int grating, long* value, int* ret = nullptr);
bool ReadBothGratingPositions(long* grating1, long* grating2);
bool CaptureForceFeedbackSample(long grating1,
    long grating2,
    long axis2ActualTorque,
    long axis4ActualTorque,
    long axis2TorqueCommand,
    long axis4TorqueCommand);
bool ReadGratingSensorTriggeredInternal(int grating, bool* triggered, unsigned short* level = nullptr);

#endif // MOTORINTERNAL_H
