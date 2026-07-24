#include "MotorInternal.h"
#include <Windows.h>

#pragma execution_character_set("utf-8")

MultiCard g_MultiCard;
std::atomic_bool g_bFollowRunning(false);
std::atomic_bool g_cardOpened(false);
std::atomic_bool g_forceFeedbackStopRequested(false);
std::atomic_long g_forceFeedbackTargetVel(0);
std::atomic_long g_forceFeedbackTargetGrating2(0);
std::atomic_long g_forceFeedbackCurrentVel(0);
std::atomic_llong g_forceFeedbackExecMaxUs(0);
std::atomic_llong g_forceFeedbackGratingReadMaxUs(0);
std::atomic_llong g_forceFeedbackActualTorqueReadMaxUs(0);
std::atomic_llong g_forceFeedbackMotorSampleMaxUs(0);
std::atomic_llong g_forceFeedbackTorqueWriteMaxUs(0);
std::atomic_llong g_forceFeedbackUpdateMaxUs(0);
std::thread g_forceFeedbackThread;
std::atomic_bool g_gratingZeroRunning(false);
std::thread g_gratingZeroThread;
std::atomic_bool g_gratingClosedLoopRunning[3] = {};
std::thread g_gratingClosedLoopThread[3];

QTextBrowser* g_logWidget = nullptr;  // 先初始化为 nullptr
AxisWorkMode g_axisMode[kMaxAxisCount + 1] = {};
std::atomic_int g_activeAxisCount(0);
std::atomic_bool g_cardClosing(false);
std::atomic_long g_sampleTorqueCache[kMaxAxisCount] = {};
int g_nextSampleTorqueAxis = 1;
bool g_grating1ReadErrorLogged = false;
bool g_grating2ReadErrorLogged = false;
int g_grating2EncoderIndex = 3;
long g_grating2LastValue = 0;
int g_grating2UnchangedCount = 0;
std::atomic_long g_gratingOffset1(0);
std::atomic_long g_gratingOffset2(0);
std::atomic_long g_uiGrating1(0);
std::atomic_long g_uiGrating2(0);
std::atomic_bool g_uiGratingSampleValid(false);
std::mutex g_forceFeedbackSampleMutex;
MotorSample g_forceFeedbackSample;
bool g_forceFeedbackSampleValid = false;

namespace {

bool WaitEtherCATInitComplete(int timeoutMs)
{
    short currentSlave = -1;
    short mode = 0;
    short modeStep = 0;
    int ret = -1;
    int modesRet = -1;
    long supportedModes = 0;
    const ULONGLONG startTick = GetTickCount64();

    do
    {
        ret = g_MultiCard.MC_ECatGetInitStep(
            &currentSlave, &mode, &modeStep);
        if (ret == 0 && currentSlave == 0)
        {
            supportedModes =
                ReadSdoLong(1, 0x6502, 0x00, &modesRet);
            if (modesRet == 0 && supportedModes != 0)
            {
                logMessage(QStringLiteral("EtherCAT初始化完成，耗时=%1 ms。")
                    .arg(GetTickCount64() - startTick + 600));
                return true;
            }
        }
        Sleep(100);
    } while (GetTickCount64() - startTick < (ULONGLONG)timeoutMs);

    logMessage(QStringLiteral("EtherCAT初始化等待超时：返回值=%1，当前站=%2，模式=%3，子步骤=%4，轴1支持模式=0x%5(ret=%6)。")
        .arg(ret)
        .arg(currentSlave)
        .arg(mode)
        .arg(modeStep)
        .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
        .arg(modesRet));
    return false;
}

}

void ResetFastSampleCache()
{
    for (int i = 0; i < kMaxAxisCount; i++)
        g_sampleTorqueCache[i] = 0;

    g_nextSampleTorqueAxis = 1;
    g_grating1ReadErrorLogged = false;
    g_grating2ReadErrorLogged = false;
    g_grating2EncoderIndex = 3;
    g_grating2LastValue = 0;
    g_grating2UnchangedCount = 0;
    g_uiGrating1 = 0;
    g_uiGrating2 = 0;
    g_uiGratingSampleValid = false;
    {
        std::lock_guard<std::mutex> lock(g_forceFeedbackSampleMutex);
        g_forceFeedbackSample = MotorSample();
        g_forceFeedbackSampleValid = false;
    }
}

bool IsValidAxis(int axis)
{
    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
        activeAxisCount = kMaxAxisCount;

    return axis >= 1 && axis <= activeAxisCount;
}

bool CanAccessEtherCAT()
{
    return g_cardOpened.load();
}
void OpenCard()
{
    char PCip[] = "192.168.0.200";
    char cardIp[] = "192.168.0.1";
    int res = 0;

    if (g_cardOpened.load())
    {
        logMessage(QStringLiteral("控制卡已打开，先执行一次安全关闭。"));
        CloseCard();
        Sleep(200);
    }
    else
    {
        // 清理上一次打开失败后可能残留在库内部的句柄。
        g_MultiCard.MC_Close();
        Sleep(100);
    }

    g_cardClosing = false;
    res = g_MultiCard.MC_Open(1, PCip, 60000, cardIp, 60000);
    if (res != 0)
    {
        logMessage(QStringLiteral("Open card failed，返回值=%1。").arg(res));
        return;
    }
    logMessage(QStringLiteral("Open card success."));

    g_MultiCard.MC_SetCommuTimer(3);
    Sleep(100);
    res = g_MultiCard.MC_ECatInit();
    if (res != 0)
    {
        logMessage(QStringLiteral("EtherCAT init failed，返回值=%1。").arg(res));
        CloseCard();
        return;
    }
    Sleep(600);
    if (!WaitEtherCATInitComplete(45000))
    {
        CloseCard();
        return;
    }

    short slaveCount = 0;
    res = g_MultiCard.MC_ECatGetSlaveCount(&slaveCount);
    if (res != 0 || slaveCount <= 0)
    {
        logMessage(QStringLiteral("Servo count check failed，返回值=%1，检测到数量=%2。").arg(res).arg(slaveCount));
        CloseCard();
        return;
    }

    int activeAxisCount = slaveCount > kMaxAxisCount ? kMaxAxisCount : slaveCount;
    g_activeAxisCount = activeAxisCount;
    ResetFastSampleCache();

    if (slaveCount > kMaxAxisCount)
    {
        logMessage(QStringLiteral("检测到%1个从站，程序最多启用前%2个轴。").arg(slaveCount).arg(kMaxAxisCount));
    }
    else
    {
        logMessage(QStringLiteral("检测到%1个从站，启用%2个轴。").arg(slaveCount).arg(activeAxisCount));
    }

    for (int axis = 1; axis <= activeAxisCount; axis++)
    {
        int loadRet = g_MultiCard.MC_ECatLoadPDOConfig(axis);
        if (loadRet != 0)
            logMessage(QStringLiteral("轴%1：PDO加载失败，返回值=%2").arg(axis).arg(loadRet));
        res += loadRet;
    }

    if (res != 0)
    {
        logMessage(QStringLiteral("PDO init failed，返回值=%1。").arg(res));
        CloseCard();
        return;
    }

    if (!ConfigServo())
    {
        logMessage(QStringLiteral("Servo config failed."));
        CloseCard();
        return;
    }

    g_cardOpened = true;
    logMessage(QStringLiteral("Servo config success."));
}

void CloseCard()
{
    if (g_cardClosing.exchange(true))
        return;

    if (g_bFollowRunning || g_forceFeedbackThread.joinable())
        StopForceFeedback();
    if (g_gratingZeroRunning || g_gratingZeroThread.joinable())
    {
        g_gratingZeroRunning = false;
        if (g_gratingZeroThread.joinable())
            g_gratingZeroThread.join();
    }
    StopAllGratingClosedLoop();

    DisableAllAxesForClose();
    Sleep(100);

    g_cardOpened = false;
    for (int axis = 1; axis <= kMaxAxisCount; axis++)
        g_axisMode[axis] = AxisModeUnknown;
    g_activeAxisCount = 0;
    ResetFastSampleCache();

    g_MultiCard.MC_Close();
    Sleep(200);
    g_cardClosing = false;
    logMessage(QStringLiteral("Card closed."));
}

bool IsCardOpened()
{
    return g_cardOpened.load() && !g_cardClosing.load();
}

