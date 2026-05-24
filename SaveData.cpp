#include "SaveData.h"

#include <QMutex>
#include <QMutexLocker>

#include "MotorController.h"

namespace {
QFile* g_file = nullptr;
QTextStream* g_stream = nullptr;
bool g_recording = false;
QMutex g_recordMutex;
}

void StartRecord()
{
    QMutexLocker locker(&g_recordMutex);

    if (g_recording)
        return;

    QString fileName =
        "Data_" +
        QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".txt";

    g_file = new QFile(fileName);
    if (!g_file->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        delete g_file;
        g_file = nullptr;
        logMessage(QStringLiteral("文件打开失败！"));
        return;
    }

    g_stream = new QTextStream(g_file);
    (*g_stream) << "time"
        << " motor1_pos motor1_vel motor1_torque"
        << " motor2_pos motor2_vel motor2_torque"
        << " motor3_pos motor3_vel motor3_torque"
        << " motor4_pos motor4_vel motor4_torque\n";
    g_recording = true;

    logMessage(QStringLiteral("开始写入: ") + fileName);
}

void StopRecord()
{
    QMutexLocker locker(&g_recordMutex);

    if (!g_recording)
        return;

    g_recording = false;

    if (g_stream)
    {
        g_stream->flush();
        delete g_stream;
        g_stream = nullptr;
    }

    if (g_file)
    {
        g_file->close();
        delete g_file;
        g_file = nullptr;
    }

    logMessage(QStringLiteral("停止写入"));
}

void RecordDataSample(long long timeIndex,
    long pos1,
    long vel1,
    long torque1,
    long pos2,
    long vel2,
    long torque2,
    long pos3,
    long vel3,
    long torque3,
    long pos4,
    long vel4,
    long torque4)
{
    QMutexLocker locker(&g_recordMutex);

    if (!g_recording || !g_stream)
        return;

    (*g_stream) << timeIndex
        << " " << pos1
        << " " << vel1
        << " " << torque1
        << " " << pos2
        << " " << vel2
        << " " << torque2
        << " " << pos3
        << " " << vel3
        << " " << torque3
        << " " << pos4
        << " " << vel4
        << " " << torque4
        << "\n";
}
