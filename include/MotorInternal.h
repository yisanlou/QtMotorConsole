#ifndef MOTORINTERNAL_H
#define MOTORINTERNAL_H

#include "MotorController.h"
#include "MultiCardCPP.h"
#include <atomic>
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
constexpr int kMaxGratingEncoderIndex = 8;

extern MultiCard g_MultiCard;
extern std::atomic_bool g_bFollowRunning;
extern std::atomic_bool g_cardOpened;
extern std::atomic_bool g_forceFeedbackStopRequested;
extern std::atomic_long g_forceFeedbackTargetVel;
extern std::atomic_long g_forceFeedbackTargetGrating2;
extern std::atomic_long g_forceFeedbackCurrentVel;
extern std::atomic_llong g_forceFeedbackExecMaxUs;
extern std::thread g_forceFeedbackThread;
extern std::atomic_bool g_gratingZeroRunning;
extern std::thread g_gratingZeroThread;
extern std::atomic_bool g_gratingClosedLoopRunning[3];
extern std::thread g_gratingClosedLoopThread[3];
extern AxisWorkMode g_axisMode[kMaxAxisCount + 1];
extern std::atomic_int g_activeAxisCount;
extern std::atomic_bool g_cardClosing;
extern long g_sampleTorqueCache[kMaxAxisCount];
extern int g_nextSampleTorqueAxis;
extern bool g_grating1ReadErrorLogged;
extern bool g_grating2ReadErrorLogged;
extern int g_grating2EncoderIndex;
extern int g_nextGrating2ProbeIndex;
extern long g_grating2LastValue;
extern int g_grating2UnchangedCount;
extern long g_gratingProbeLastValue[kMaxGratingEncoderIndex + 1];
extern bool g_gratingProbeHasValue[kMaxGratingEncoderIndex + 1];
extern std::atomic_long g_gratingOffset1;
extern std::atomic_long g_gratingOffset2;

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
bool ReadGratingSensorTriggeredInternal(int grating, bool* triggered, unsigned short* level = nullptr);

#endif // MOTORINTERNAL_H
