#include "MotorInternal.h"
#include <QDateTime>
#include <QMetaObject>
#include <QPointer>

#pragma execution_character_set("utf-8")

void logMessage(const QString& msg)
{
    QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString fullMsg = QString("[%1] %2").arg(timeStr, msg);

    QPointer<QTextBrowser> logWidget = g_logWidget;
    if (logWidget)
    {
        QMetaObject::invokeMethod(logWidget,
            [logWidget, fullMsg]() {
                if (logWidget)
                    logWidget->append(fullMsg);
            },
            Qt::AutoConnection);
    }

    qDebug().noquote() << fullMsg;
}
