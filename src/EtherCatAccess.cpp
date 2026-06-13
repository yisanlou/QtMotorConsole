#include "MotorInternal.h"

#pragma execution_character_set("utf-8")

long ReadSdoLong(int axis, int index, int subIndex, int* ret)
{
    long value = 0;
    int localRet = g_MultiCard.MC_ECatGetSdoValue(axis, index, subIndex, &value);
    if (ret)
        *ret = localRet;
    return value;
}

long ReadOperationMode(int axis)
{
    if (!IsValidAxis(axis))
        return 0;

    return ReadSdoLong(axis, 0x6061, 0x00);
}

int SetSdoWithLog(int axis, int index, int subIndex, long value, short len, const QString& name)
{
    int ret = g_MultiCard.MC_ECatSetSdoValue(axis, index, subIndex, value, len);
    if (ret != 0)
    {
        logMessage(QStringLiteral("轴%1：写%2(0x%3:%4=%5)失败，返回值=%6")
            .arg(axis)
            .arg(name)
            .arg(index, 4, 16, QChar('0'))
            .arg(subIndex)
            .arg(value)
            .arg(ret));
    }

    return ret;
}

short ReadStatusWord(int axis, int* ret)
{
    short statusWord = 0;
    int localRet = g_MultiCard.MC_ECatGetStatusWord(axis, &statusWord);
    if (ret)
        *ret = localRet;
    return statusWord;
}

int AddStepResult(int total, const QString& modeName, const QString& stepName, int ret)
{
    if (ret != 0)
        logMessage(QStringLiteral("%1：%2失败，返回值=%3").arg(modeName, stepName).arg(ret));

    return total + ret;
}

