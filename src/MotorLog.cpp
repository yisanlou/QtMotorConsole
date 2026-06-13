#include "MotorInternal.h"
#include <QDateTime>

#pragma execution_character_set("utf-8")

void logMessage(const QString& msg)
{
    QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString fullMsg = QString("[%1] %2").arg(timeStr, msg);

    if (g_logWidget)
        g_logWidget->append(fullMsg);  // 中文正常显示

    qDebug().noquote() << fullMsg;
}
