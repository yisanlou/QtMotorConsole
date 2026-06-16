#include "MotorInternal.h"
#include <Windows.h>

#pragma execution_character_set("utf-8")

bool ConfigServo()
{
    bool allAxesOnline = true;

    int activeAxisCount = g_activeAxisCount.load();
    if (activeAxisCount <= 0)
    {
        logMessage(QStringLiteral("Servo config failed：没有可用轴。"));
        return false;
    }

    for (int axis = 1; axis <= activeAxisCount; axis++)
    {
        long supportedModes = 0;
        long displayMode = 0;
        short statusWord = 0;

        if (!WaitAxisOnline(axis, &supportedModes, &displayMode, &statusWord))
        {
            logMessage(QStringLiteral("轴%1：EtherCAT从站未就绪，支持模式=0x%2，当前模式=0x%3，状态字=0x%4")
                .arg(axis)
                .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
                .arg((unsigned long)displayMode, 2, 16, QChar('0'))
                .arg((unsigned short)statusWord, 4, 16, QChar('0')));
            allAxesOnline = false;
            continue;
        }

        int modeRet = g_MultiCard.MC_ECatSetCtrlMode(axis, kModeCsp);
        int directRet = 0;
        if (modeRet != 0)
        {
            directRet = g_MultiCard.MC_ECatSetSdoValue(axis, 0x6060, 0x00, kModeCsp, 1);
            if (directRet != 0)
            {
                logMessage(QStringLiteral("轴%1：设置初始CSP模式返回异常，控制卡返回=%2，直接写6060h返回=%3，将以读回模式为准")
                    .arg(axis)
                    .arg(modeRet)
                    .arg(directRet));
            }
        }

        Sleep(30);
        displayMode = ReadOperationMode(axis);
        statusWord = ReadStatusWord(axis);

        if (modeRet != 0 && directRet != 0 && displayMode == kModeCsp)
        {
            logMessage(QStringLiteral("轴%1：读回已处于CSP模式，忽略初始模式写入返回异常。")
                .arg(axis));
        }

        logMessage(QStringLiteral("轴%1：EtherCAT就绪，支持模式=0x%2，当前模式=0x%3，状态字=0x%4")
            .arg(axis)
            .arg((unsigned long)supportedModes, 8, 16, QChar('0'))
            .arg((unsigned long)displayMode, 2, 16, QChar('0'))
            .arg((unsigned short)statusWord, 4, 16, QChar('0')));
    }

    return allAxesOnline;
}



