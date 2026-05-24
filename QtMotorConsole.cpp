#include "QtMotorConsole.h"
#include "MotorController.h"
#include "SaveData.h"
#include <QThread>
#include <QDebug>
#include <QMessageBox>

#include <QPainterPath>
#include <QPen>
#include <QGraphicsPathItem>
#include <QGraphicsSimpleTextItem>
#include <QBrush>
#include <QFont>
#include <QTimer>
#include <QIntValidator>

#include <windows.h>
#include <cstdio>
#include <QDateTime>
#include <QTextStream>
#include <chrono>
#include <cmath>
#include <iostream>

namespace {
const int kMaxPointCount = 500;
const double kSceneWidth = 590.0;
const double kSceneHeight = 380.0;
const double kLeftMargin = 55.0;
const double kRightMargin = 55.0;
const double kTopMargin = 20.0;
const double kBottomMargin = 35.0;
const double kPlotWidth = kSceneWidth - kLeftMargin - kRightMargin;
const double kPlotHeight = kSceneHeight - kTopMargin - kBottomMargin;
const double kVelocityDisplayScale = 1.0;
const double kTorqueDisplayScale = 1.0;
const double kPositionDisplayStep = 1.0; // kpulse
const double kVelocityDisplayStep = 1.0; // pulse/s
const double kTorqueDisplayStep = 1.0;   // 0.1%
const double kFilterLastWeight = 0.8;
const double kFilterCurrentWeight = 0.2;
const double kVelocityFilterLastWeight = 0.85;
const double kVelocityFilterCurrentWeight = 0.15;
const int kVelocityAverageWindow = 5;
const int kTorqueAverageWindow = 5;

double quantizeDisplayValue(double value, double step)
{
    if (step <= 0.0)
        return value;

    return std::round(value / step) * step;
}

void roundDisplayRange(double* yMin, double* yMax, double step)
{
    if (step <= 0.0)
        return;

    *yMin = std::floor(*yMin / step) * step;
    *yMax = std::ceil(*yMax / step) * step;
    if (*yMax <= *yMin)
        *yMax = *yMin + step;
}

double trimmedMean(const double* values, int count)
{
    if (count <= 0)
        return 0.0;

    double sum = 0.0;
    double minValue = values[0];
    double maxValue = values[0];
    for (int i = 0; i < count; i++)
    {
        const double value = values[i];
        sum += value;
        if (value < minValue)
            minValue = value;
        if (value > maxValue)
            maxValue = value;
    }

    if (count >= 5)
        return (sum - minValue - maxValue) / (count - 2);

    return sum / count;
}

}

QtMotorConsole::QtMotorConsole(QWidget* parent)
    : QWidget(parent),
    m_scene(nullptr),
    m_axis(nullptr),
    m_posCurve(nullptr),
    m_velCurve(nullptr),
    m_torqueCurve(nullptr),
    m_timer(nullptr),
    m_perfTimer(nullptr),
    m_timeIndex(0),
    m_selectedAxis(1),
    m_lastRenderedSampleIndex(-1),
    m_uiExecMaxUs(0),
    m_filterValid(false),
    m_filteredVel(0.0),
    m_filteredTorque(0.0),
    m_velAverageBuffer{ 0.0, 0.0, 0.0, 0.0, 0.0 },
    m_torqueAverageBuffer{ 0.0, 0.0, 0.0, 0.0, 0.0 },
    m_velAverageIndex(0),
    m_velAverageCount(0),
    m_torqueAverageIndex(0),
    m_torqueAverageCount(0),
    m_sampleRunning(false),
    m_sampleExecMaxUs(0)
{
    ui.setupUi(this);
    g_logWidget = ui.textBrowser_Log;
    logMessage(QStringLiteral("Start"));
    setupUiState();
    setupPlotScene();
    setupConnections();
}

void QtMotorConsole::setupUiState()
{
    ui.lineEdit_ForceVel->setValidator(new QIntValidator(-5000, 5000, this));
    ui.lineEdit_PosTarget->setValidator(new QIntValidator(-1000000000, 1000000000, this));
    ui.lineEdit_PosSpeed->setValidator(new QIntValidator(0, 1000000, this));
    ui.lineEdit_VelTarget->setValidator(new QIntValidator(-1000000, 1000000, this));
    ui.lineEdit_TorqueTarget->setValidator(new QIntValidator(-1000, 1000, this));
    ui.checkBox_Vel->setText(QStringLiteral("速度(pulse/s)"));
    ui.checkBox_Torque->setText(QStringLiteral("力矩(0.1%)"));
}

void QtMotorConsole::setupPlotScene()
{
    m_scene = new QGraphicsScene(this);
    ui.graphicsView_wave->setScene(m_scene);
    ui.graphicsView_wave->setRenderHint(QPainter::Antialiasing);
    ui.graphicsView_wave->setBackgroundBrush(QBrush(Qt::white));
    ui.graphicsView_wave->setStyleSheet(QStringLiteral("QGraphicsView { border: 1px solid #9a9a9a; background: white; }"));

    m_axis = m_scene->addPath(QPainterPath(), QPen(Qt::darkGray, 1));
    m_axis->setZValue(0);
    m_posCurve = m_scene->addPath(QPainterPath(), QPen(Qt::green, 2));
    m_velCurve = m_scene->addPath(QPainterPath(), QPen(Qt::blue, 2));
    m_torqueCurve = m_scene->addPath(QPainterPath(), QPen(Qt::red, 2));
    m_posCurve->setZValue(1);
    m_velCurve->setZValue(1);
    m_torqueCurve->setZValue(1);
    m_scene->setSceneRect(0, 0, kSceneWidth, kSceneHeight);
    drawAxis(0.0, 1.0, 0.0, 1.0, false, QStringLiteral("M1 Pos (kpulse)"), QBrush(Qt::darkGreen));

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &QtMotorConsole::updateWave);
    m_timer->start(10);

    m_perfTimer = new QTimer(this);
    connect(m_perfTimer, &QTimer::timeout, this, &QtMotorConsole::updatePerformancePanel);
    m_perfTimer->start(1000);
}

void QtMotorConsole::setupConnections()
{
    connect(ui.pushButton_OpenCard, &QPushButton::clicked, this, [this]() {
        OpenCard();
        if (IsCardOpened())
            startSampleThread();
    });

    connect(ui.pushButton_Stop, &QPushButton::clicked, this, [this]() {
        stopSampleThread();
        CloseCard();
    });

    connect(ui.pushButton_StartForce, &QPushButton::clicked, this, [this]() {
        StartForceFeedback(ui.lineEdit_ForceVel->text().toLong());
    });

    connect(ui.pushButton_StopForce, &QPushButton::clicked, this, []() {
        StopForceFeedback();
    });

    connect(ui.lineEdit_ForceVel, &QLineEdit::returnPressed, this, [this]() {
        StartForceFeedback(ui.lineEdit_ForceVel->text().toLong());
    });

    connect(ui.checkBox_Torque, &QCheckBox::toggled, this, [this](bool checked) {
        if (checked)
            ui.checkBox_Pos->setChecked(false);
    });

    connect(ui.checkBox_Pos, &QCheckBox::toggled, this, [this](bool checked) {
        if (checked)
            ui.checkBox_Torque->setChecked(false);
    });

    connect(ui.pushButton_PosMove, &QPushButton::clicked, this, [this]() {
        PositionModeMove(selectedAxis(), ui.lineEdit_PosTarget->text().toLong(), ui.lineEdit_PosSpeed->text().toLong());
    });

    connect(ui.pushButton_VelMove, &QPushButton::clicked, this, [this]() {
        VelocityModeMove(selectedAxis(), ui.lineEdit_VelTarget->text().toLong());
    });

    connect(ui.pushButton_TorqueSet, &QPushButton::clicked, this, [this]() {
        TorqueModeSet(selectedAxis(), ui.lineEdit_TorqueTarget->text().toLong());
    });

    connect(ui.pushButton_DisableAxis, &QPushButton::clicked, this, []() {
        DisableAxis(1);
        DisableAxis(2);
    });

    connect(ui.pushButton_DisableMotor1, &QPushButton::clicked, this, []() {
        DisableAxis(1);
    });

    connect(ui.pushButton_DisableMotor2, &QPushButton::clicked, this, []() {
        DisableAxis(2);
    });

    connect(ui.pushButton_StartRecord, &QPushButton::clicked, this, []() {
        StartRecord();
    });

    connect(ui.pushButton_StopRecord, &QPushButton::clicked, this, []() {
        StopRecord();
    });
}

void QtMotorConsole::resetWaveBuffers()
{
    m_timeIndex = 0;
    m_lastRenderedSampleIndex = -1;
    m_posPoints.clear();
    m_velPoints.clear();
    m_torquePoints.clear();
    m_posCurve->setPath(QPainterPath());
    m_velCurve->setPath(QPainterPath());
    m_torqueCurve->setPath(QPainterPath());
    m_filterValid = false;
    m_filteredVel = 0.0;
    m_filteredTorque = 0.0;
    for (int i = 0; i < kVelocityAverageWindow; i++)
        m_velAverageBuffer[i] = 0.0;
    for (int i = 0; i < kTorqueAverageWindow; i++)
        m_torqueAverageBuffer[i] = 0.0;
    m_velAverageIndex = 0;
    m_velAverageCount = 0;
    m_torqueAverageIndex = 0;
    m_torqueAverageCount = 0;
}

int QtMotorConsole::selectedAxis() const
{
    return ui.radioButton_Motor2->isChecked() ? 2 : 1;
}

void QtMotorConsole::updateWave()
{
    auto execStart = std::chrono::steady_clock::now();
    auto finalizeExecTime = [this, &execStart]() {
        auto execEnd = std::chrono::steady_clock::now();
        long long execUs = std::chrono::duration_cast<std::chrono::microseconds>(execEnd - execStart).count();
        if (execUs > m_uiExecMaxUs)
            m_uiExecMaxUs = execUs;
    };

    SampleState sample;
    {
        std::lock_guard<std::mutex> lock(m_sampleMutex);
        sample = m_latestSample;
    }

    if (!sample.valid || sample.sampleIndex == m_lastRenderedSampleIndex)
    {
        finalizeExecTime();
        return;
    }

    int axis = selectedAxis();
    if (axis != m_selectedAxis)
    {
        m_selectedAxis = axis;
        resetWaveBuffers();
    }

    if (sample.sampleIndex == m_lastRenderedSampleIndex)
    {
        finalizeExecTime();
        return;
    }

    m_lastRenderedSampleIndex = sample.sampleIndex;

    long pos = axis == 2 ? sample.pos2 : sample.pos1;
    long vel = axis == 2 ? sample.vel2 : sample.vel1;
    long torque = axis == 2 ? sample.torque2 : sample.torque1;

    RecordDataSample(sample.sampleIndex,
        sample.pos1,
        sample.vel1,
        sample.torque1,
        sample.pos2,
        sample.vel2,
        sample.torque2);

    m_timeIndex++;
    appendWavePoint(&m_posPoints, quantizeDisplayValue(pos * 0.001, kPositionDisplayStep));
    double scaledVel = vel * kVelocityDisplayScale;
    double scaledTorque = torque * kTorqueDisplayScale;
    m_velAverageBuffer[m_velAverageIndex] = scaledVel;
    m_velAverageIndex = (m_velAverageIndex + 1) % kVelocityAverageWindow;
    if (m_velAverageCount < kVelocityAverageWindow)
        m_velAverageCount++;
    m_torqueAverageBuffer[m_torqueAverageIndex] = scaledTorque;
    m_torqueAverageIndex = (m_torqueAverageIndex + 1) % kTorqueAverageWindow;
    if (m_torqueAverageCount < kTorqueAverageWindow)
        m_torqueAverageCount++;

    double filteredInputVel = trimmedMean(m_velAverageBuffer, m_velAverageCount);
    double filteredInputTorque = trimmedMean(m_torqueAverageBuffer, m_torqueAverageCount);

    if (!m_filterValid)
    {
        m_filteredVel = filteredInputVel;
        m_filteredTorque = filteredInputTorque;
        m_filterValid = true;
    }
    else
    {
        m_filteredVel = m_filteredVel * kVelocityFilterLastWeight + filteredInputVel * kVelocityFilterCurrentWeight;
        m_filteredTorque = m_filteredTorque * kFilterLastWeight + filteredInputTorque * kFilterCurrentWeight;
    }

    appendWavePoint(&m_velPoints, quantizeDisplayValue(m_filteredVel, kVelocityDisplayStep));
    appendWavePoint(&m_torquePoints, quantizeDisplayValue(m_filteredTorque, kTorqueDisplayStep));

    bool showPos = ui.checkBox_Pos->isChecked();
    bool showVel = ui.checkBox_Vel->isChecked();
    bool showTorque = ui.checkBox_Torque->isChecked();

    if (showPos && showTorque)
    {
        showPos = false;
        ui.checkBox_Pos->setChecked(false);
    }

    m_posCurve->setVisible(showPos);
    m_velCurve->setVisible(showVel);
    m_torqueCurve->setVisible(showTorque);

    double leftMin = 0.0;
    double leftMax = 0.0;
    bool hasLeftValue = false;
    QString leftTitle = QString("M%1 Pos (kpulse)").arg(m_selectedAxis);
    QBrush leftBrush(Qt::darkGreen);

    if (showTorque)
    {
        leftTitle = QString("M%1 Torque (0.1%)").arg(m_selectedAxis);
        leftBrush = QBrush(Qt::red);
        updateValueRange(&m_torquePoints, &leftMin, &leftMax, &hasLeftValue);
    }
    else if (showPos)
    {
        updateValueRange(&m_posPoints, &leftMin, &leftMax, &hasLeftValue);
    }

    if (!hasLeftValue)
    {
        leftMin = 0.0;
        leftMax = 1.0;
    }
    addRangePadding(&leftMin, &leftMax);
    if (showTorque)
        roundDisplayRange(&leftMin, &leftMax, kTorqueDisplayStep);
    else if (showPos)
        roundDisplayRange(&leftMin, &leftMax, kPositionDisplayStep);

    double rightMin = 0.0;
    double rightMax = 0.0;
    bool hasRightValue = false;
    if (showVel)
        updateValueRange(&m_velPoints, &rightMin, &rightMax, &hasRightValue);
    if (!hasRightValue)
    {
        rightMin = 0.0;
        rightMax = 1.0;
    }
    addRangePadding(&rightMin, &rightMax);
    if (showVel)
        roundDisplayRange(&rightMin, &rightMax, kVelocityDisplayStep);

    drawAxis(leftMin, leftMax, rightMin, rightMax, showVel, leftTitle, leftBrush);
    drawWaveCurve(m_posCurve, &m_posPoints, leftMin, leftMax);
    drawWaveCurve(m_velCurve, &m_velPoints, rightMin, rightMax);
    drawWaveCurve(m_torqueCurve, &m_torquePoints, leftMin, leftMax);

    m_scene->setSceneRect(0, 0, kSceneWidth, kSceneHeight);
    finalizeExecTime();
}
void QtMotorConsole::appendWavePoint(QVector<QPointF>* points, double value)
{
    points->append(QPointF(m_timeIndex, value));

    if (points->size() > kMaxPointCount)
        points->removeFirst();
}

void QtMotorConsole::updateValueRange(const QVector<QPointF>* points, double* yMin, double* yMax, bool* hasValue)
{
    for (int i = 0; i < points->size(); i++)
    {
        double value = points->at(i).y();
        if (!(*hasValue))
        {
            *yMin = value;
            *yMax = value;
            *hasValue = true;
        }
        else
        {
            if (value < *yMin)
                *yMin = value;
            if (value > *yMax)
                *yMax = value;
        }
    }
}

void QtMotorConsole::addRangePadding(double* yMin, double* yMax)
{
    double yRange = *yMax - *yMin;
    if (yRange < 1.0)
    {
        *yMin -= 0.5;
        *yMax += 0.5;
    }
    else
    {
        double padding = yRange * 0.1;
        *yMin -= padding;
        *yMax += padding;
    }
}

void QtMotorConsole::drawWaveCurve(QGraphicsPathItem* curve, const QVector<QPointF>* points, double yMin, double yMax)
{
    if (!curve->isVisible())
    {
        curve->setPath(QPainterPath());
        return;
    }

    double yRange = yMax - yMin;
    if (yRange < 1.0)
        yRange = 1.0;

    double xMin = m_timeIndex - kMaxPointCount + 1;
    if (xMin < 1.0)
        xMin = 1.0;

    QPainterPath path;
    for (int i = 0; i < points->size(); i++)
    {
        double xRatio = (points->at(i).x() - xMin) / (kMaxPointCount - 1);
        double yRatio = (yMax - points->at(i).y()) / yRange;
        double x = kLeftMargin + xRatio * kPlotWidth;
        double y = kTopMargin + yRatio * kPlotHeight;

        if (i == 0)
            path.moveTo(x, y);
        else
            path.lineTo(x, y);
    }

    curve->setPath(path);
}

void QtMotorConsole::drawAxis(double leftMin, double leftMax, double rightMin, double rightMax, bool showRightAxis, const QString& leftTitleText, const QBrush& leftBrush)
{
    QPainterPath axisPath;
    const double x0 = kLeftMargin;
    const double x1 = kLeftMargin + kPlotWidth;
    const double y0 = kTopMargin + kPlotHeight;
    const int xTickCount = 5;
    const int yTickCount = 5;
    int labelIndex = 0;

    axisPath.moveTo(x0, kTopMargin);
    axisPath.lineTo(x0, y0);
    axisPath.lineTo(x1, y0);

    if (showRightAxis)
    {
        axisPath.moveTo(x1, kTopMargin);
        axisPath.lineTo(x1, y0);
    }

    double xMin = m_timeIndex - kMaxPointCount + 1;
    if (xMin < 1.0)
        xMin = 1.0;

    for (int i = 0; i <= xTickCount; i++)
    {
        double x = kLeftMargin + kPlotWidth * i / xTickCount;
        int sampleIndex = (int)(xMin + (kMaxPointCount - 1) * i / xTickCount);

        axisPath.moveTo(x, y0);
        axisPath.lineTo(x, y0 + 5.0);

        QGraphicsSimpleTextItem* label = axisLabel(labelIndex++);
        label->setText(QString::number(sampleIndex));
        label->setBrush(QBrush(Qt::darkGray));
        label->setZValue(2);
        label->setPos(x - 15.0, y0 + 8.0);
    }

    for (int i = 0; i <= yTickCount; i++)
    {
        double y = kTopMargin + kPlotHeight * i / yTickCount;
        double leftValue = leftMax - (leftMax - leftMin) * i / yTickCount;
        int leftDecimals = 2;
        if (leftTitleText.contains(QStringLiteral("Torque")))
            leftDecimals = 0;
        else if (leftTitleText.contains(QStringLiteral("Pos")))
            leftDecimals = 0;

        axisPath.moveTo(x0 - 5.0, y);
        axisPath.lineTo(x0, y);

        QGraphicsSimpleTextItem* leftLabel = axisLabel(labelIndex++);
        leftLabel->setText(QString::number(leftValue, 'f', leftDecimals));
        leftLabel->setBrush(leftBrush);
        leftLabel->setZValue(2);
        leftLabel->setPos(2.0, y - 8.0);

        if (showRightAxis)
        {
            double rightValue = rightMax - (rightMax - rightMin) * i / yTickCount;

            axisPath.moveTo(x1, y);
            axisPath.lineTo(x1 + 5.0, y);

            QGraphicsSimpleTextItem* rightLabel = axisLabel(labelIndex++);
            rightLabel->setText(QString::number(rightValue, 'f', 0));
            rightLabel->setBrush(QBrush(Qt::blue));
            rightLabel->setZValue(2);
            rightLabel->setPos(x1 + 8.0, y - 8.0);
        }
    }

    QGraphicsSimpleTextItem* leftTitle = axisLabel(labelIndex++);
    leftTitle->setText(leftTitleText);
    leftTitle->setBrush(leftBrush);
    leftTitle->setZValue(2);
    leftTitle->setPos(2.0, 2.0);

    if (showRightAxis)
    {
        QGraphicsSimpleTextItem* rightTitle = axisLabel(labelIndex++);
        rightTitle->setText(QStringLiteral("Vel (pulse/s)"));
        rightTitle->setBrush(QBrush(Qt::blue));
        rightTitle->setZValue(2);
        rightTitle->setPos(x1 + 8.0, 2.0);
    }

    for (int i = labelIndex; i < m_axisLabels.size(); i++)
        m_axisLabels[i]->setVisible(false);

    m_axis->setPath(axisPath);
}

QGraphicsSimpleTextItem* QtMotorConsole::axisLabel(int index)
{
    while (index >= m_axisLabels.size())
    {
        QGraphicsSimpleTextItem* label = m_scene->addSimpleText(QString(), QFont("Arial", 8));
        label->setZValue(2);
        m_axisLabels.append(label);
    }

    QGraphicsSimpleTextItem* label = m_axisLabels[index];
    label->setVisible(true);
    return label;
}

void QtMotorConsole::startSampleThread()
{
    if (m_sampleRunning)
        return;

    m_sampleRunning = true;
    m_sampleThread = std::thread(&QtMotorConsole::sampleLoop, this);
    logMessage(QStringLiteral("采样线程已启动。"));
}

void QtMotorConsole::stopSampleThread()
{
    bool wasRunning = m_sampleRunning;
    m_sampleRunning = false;

    if (m_sampleThread.joinable())
    {
        m_sampleThread.join();
        wasRunning = true;
    }

    if (wasRunning)
        logMessage(QStringLiteral("采样线程已停止。"));
}

void QtMotorConsole::sampleLoop()
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    long long sampleIndex = 0;
    auto nextTime = std::chrono::steady_clock::now();

    while (m_sampleRunning)
    {
        auto execStart = std::chrono::steady_clock::now();
        nextTime += std::chrono::milliseconds(10);

        if (!IsCardOpened())
        {
            std::this_thread::sleep_until(nextTime);
            continue;
        }

        SampleState sample;
        sample.sampleIndex = sampleIndex++;
        MotorSample motorSample = ReadFastSample();
        sample.pos1 = motorSample.pos1;
        sample.pos2 = motorSample.pos2;
        sample.vel1 = motorSample.vel1;
        sample.vel2 = motorSample.vel2;
        sample.torque1 = motorSample.torque1;
        sample.torque2 = motorSample.torque2;
        sample.valid = true;

        {
            std::lock_guard<std::mutex> lock(m_sampleMutex);
            m_latestSample = sample;
        }

        auto execEnd = std::chrono::steady_clock::now();
        long long execUs = std::chrono::duration_cast<std::chrono::microseconds>(execEnd - execStart).count();
        long long oldMaxUs = m_sampleExecMaxUs.load();
        while (execUs > oldMaxUs && !m_sampleExecMaxUs.compare_exchange_weak(oldMaxUs, execUs))
        {
        }

        std::this_thread::sleep_until(nextTime);
    }
}

void QtMotorConsole::updatePerformancePanel()
{
    ui.label_UiExecValue->setText(QString("%1 us").arg(m_uiExecMaxUs));
    ui.label_SampleExecValue->setText(QString("%1 us").arg(m_sampleExecMaxUs.exchange(0)));
    ui.label_ForceExecValue->setText(QString("%1 us").arg(ConsumeForceFeedbackExecMaxUs()));
    m_uiExecMaxUs = 0;
}
void QtMotorConsole::closeEvent(QCloseEvent* event)
{
    stopSampleThread();
    CloseCard();
    QWidget::closeEvent(event);
}

QtMotorConsole::~QtMotorConsole()
{
    stopSampleThread();
}
