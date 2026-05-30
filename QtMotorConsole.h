#pragma once

#include <QtWidgets/QWidget>
#include "ui_QtMotorConsole.h"

#include <QGraphicsScene>
#include <QGraphicsPathItem>
#include <QGraphicsSimpleTextItem>
#include <QTimer>
#include <QVector>
#include <QCloseEvent>
#include <QBrush>
#include <atomic>
#include <mutex>
#include <thread>

class QtMotorConsole : public QWidget
{
    Q_OBJECT

public:
    QtMotorConsole(QWidget *parent = nullptr);
    ~QtMotorConsole();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    struct SampleState
    {
        long long sampleIndex = 0;
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
        bool valid = false;
    };

    Ui::QtMotorConsoleClass ui;

    QGraphicsScene* m_scene;
    QGraphicsPathItem* m_axis;
    QGraphicsPathItem* m_posCurve;
    QGraphicsPathItem* m_velCurve;
    QGraphicsPathItem* m_torqueCurve;
    QGraphicsPathItem* m_gratingCurve;
    QTimer* m_timer;
    QTimer* m_perfTimer;
    QVector<QGraphicsSimpleTextItem*> m_axisLabels;

    QVector<QPointF> m_posPoints;
    QVector<QPointF> m_velPoints;
    QVector<QPointF> m_torquePoints;
    QVector<QPointF> m_gratingPoints;
    int m_timeIndex;
    int m_selectedAxis;
    int m_selectedGrating;
    long long m_lastRenderedSampleIndex;
    long long m_uiExecMaxUs;
    bool m_filterValid;
    double m_filteredVel;
    double m_filteredTorque;
    double m_velAverageBuffer[5];
    double m_torqueAverageBuffer[5];
    int m_velAverageIndex;
    int m_velAverageCount;
    int m_torqueAverageIndex;
    int m_torqueAverageCount;
    std::thread m_sampleThread;
    std::atomic_bool m_sampleRunning;
    std::atomic_llong m_sampleExecMaxUs;
    std::mutex m_sampleMutex;
    SampleState m_latestSample;

private slots:
    void updateWave();
    void updatePerformancePanel();

private:
    void setupUiState();
    void setupPlotScene();
    void setupConnections();
    void resetWaveBuffers();
    int selectedAxis() const;
    void startSampleThread();
    void stopSampleThread();
    void sampleLoop();
    void drawAxis(double leftMin, double leftMax, double rightMin, double rightMax, bool showRightAxis, const QString& leftTitle, const QBrush& leftBrush);
    void appendWavePoint(QVector<QPointF>* points, double value);
    void updateValueRange(const QVector<QPointF>* points, double* yMin, double* yMax, bool* hasValue);
    void addRangePadding(double* yMin, double* yMax);
    void drawWaveCurve(QGraphicsPathItem* curve, const QVector<QPointF>* points, double yMin, double yMax);
    QGraphicsSimpleTextItem* axisLabel(int index);
};
