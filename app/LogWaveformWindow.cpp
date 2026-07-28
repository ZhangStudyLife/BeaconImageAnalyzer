#include "LogWaveformWindow.h"

#include "WaveformHistoryStore.h"
#include "WaveformViewport.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QDoubleSpinBox>
#include <QFontMetrics>
#include <QGridLayout>
#include <QHash>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace
{
constexpr double InvalidSentinel = -900.0;
constexpr int RefreshIntervalMs = 33;

struct PlotPoint
{
    double timeMs = 0.0;
    double value = 0.0;
    bool valid = false;
};

struct PlotSeries
{
    int channelIndex = -1;
    QString label;
    QString unit;
    QColor color;
    QVector<PlotPoint> points;
};

QColor channelColor(int channelIndex)
{
    static const QColor colors[] = {
        QColor(45, 212, 191),
        QColor(246, 212, 74),
        QColor(248, 113, 113),
        QColor(96, 165, 250),
        QColor(244, 114, 182),
        QColor(74, 222, 128),
        QColor(251, 146, 60),
        QColor(196, 181, 253),
        QColor(34, 211, 238),
        QColor(250, 204, 21),
        QColor(251, 113, 133),
        QColor(52, 211, 153)
    };
    return colors[channelIndex % (sizeof(colors) / sizeof(colors[0]))];
}

bool isUsableValue(double value)
{
    return std::isfinite(value) && value > InvalidSentinel;
}

bool isUsableChannelValue(const JustFloatLogRow& row, int channelIndex, double value)
{
    if (!isUsableValue(value))
    {
        return false;
    }
    if (channelIndex >= 1 && channelIndex <= 18)
    {
        const int offset = channelIndex - 1;
        return row.cameras[offset / 6].beacons[(offset % 6) / 3].valid;
    }
    if (channelIndex >= 19 && channelIndex <= 33)
    {
        return row.cameras[(channelIndex - 19) / 5].carLamp.valid;
    }
    if (channelIndex >= 38 && channelIndex <= 42)
    {
        return row.hasMotionData;
    }
    return true;
}

double sourceTimeMs(const JustFloatLogRow& row)
{
    if (std::isfinite(row.syncTimeMs) && row.syncTimeMs > 0.0)
    {
        return row.syncTimeMs;
    }
    if (std::isfinite(row.rowTime) && row.rowTime >= 0.0)
    {
        return row.rowTime;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

QString channelLabel(const JustFloatChannelDescriptor& descriptor)
{
    QString label = QStringLiteral("I%1 %2").arg(descriptor.index).arg(descriptor.name);
    if (!descriptor.unit.isEmpty())
    {
        label += QStringLiteral(" (%1)").arg(descriptor.unit);
    }
    return label;
}

class WaveformPlotWidget : public QWidget
{
public:
    explicit WaveformPlotWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMouseTracking(true);
        setMinimumSize(420, 280);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    int plotPixelWidth() const
    {
        return qMax(1, plotRect().width());
    }

    void setPlotData(QVector<PlotSeries> series,
                     double startTimeMs,
                     double endTimeMs,
                     double minimumValue,
                     double maximumValue)
    {
        m_series = std::move(series);
        m_startTimeMs = startTimeMs;
        m_endTimeMs = endTimeMs;
        m_minimumValue = minimumValue;
        m_maximumValue = maximumValue;
        update();
    }

    void clearPlot()
    {
        m_series.clear();
        m_startTimeMs = 0.0;
        m_endTimeMs = 10000.0;
        m_minimumValue = -1.0;
        m_maximumValue = 1.0;
        update();
    }

    void setNavigationCallbacks(std::function<void(double, double)> zoomRequested,
                                std::function<void(double, double)> panRequested,
                                std::function<void()> followRequested)
    {
        m_zoomRequested = std::move(zoomRequested);
        m_panRequested = std::move(panRequested);
        m_followRequested = std::move(followRequested);
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor(13, 15, 17));

        const QRect area = plotRect();
        painter.fillRect(area, QColor(18, 21, 24));
        drawGrid(&painter, area);
        drawSeries(&painter, area);
        drawLegend(&painter, area);
        drawCursor(&painter, area);
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        m_cursorPosition = event->position();
        m_cursorVisible = plotRect().contains(m_cursorPosition.toPoint());
        if (m_dragging && (event->buttons() & Qt::LeftButton) != 0)
        {
            const double deltaX = event->position().x() - m_lastDragPosition.x();
            m_lastDragPosition = event->position();
            if (m_panRequested)
            {
                m_panRequested(deltaX, plotRect().width());
            }
        }
        else if (m_cursorVisible)
        {
            setCursor(Qt::OpenHandCursor);
        }
        else
        {
            unsetCursor();
        }
        update();
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && plotRect().contains(event->position().toPoint()))
        {
            m_dragging = true;
            m_lastDragPosition = event->position();
            setCursor(Qt::ClosedHandCursor);
            event->accept();
            return;
        }
        QWidget::mousePressEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && m_dragging)
        {
            m_dragging = false;
            setCursor(plotRect().contains(event->position().toPoint())
                          ? Qt::OpenHandCursor
                          : Qt::ArrowCursor);
            event->accept();
            return;
        }
        QWidget::mouseReleaseEvent(event);
    }

    void mouseDoubleClickEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && plotRect().contains(event->position().toPoint()))
        {
            if (m_followRequested)
            {
                m_followRequested();
            }
            event->accept();
            return;
        }
        QWidget::mouseDoubleClickEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override
    {
        const QRect area = plotRect();
        if (!area.contains(event->position().toPoint()) || event->angleDelta().y() == 0)
        {
            QWidget::wheelEvent(event);
            return;
        }
        const double anchorRatio = qBound(0.0,
                                          (event->position().x() - area.left()) /
                                              qMax(1.0, static_cast<double>(area.width())),
                                          1.0);
        if (m_zoomRequested)
        {
            m_zoomRequested(event->angleDelta().y() / 120.0, anchorRatio);
        }
        event->accept();
    }

    void leaveEvent(QEvent*) override
    {
        m_cursorVisible = false;
        if (!m_dragging)
        {
            unsetCursor();
        }
        update();
    }

private:
    QRect plotRect() const
    {
        return rect().adjusted(64, 18, -18, -42);
    }

    double xForTime(double timeMs, const QRect& area) const
    {
        const double span = qMax(1.0, m_endTimeMs - m_startTimeMs);
        return area.left() + (timeMs - m_startTimeMs) / span * area.width();
    }

    double yForValue(double value, const QRect& area) const
    {
        const double span = qMax(1e-12, m_maximumValue - m_minimumValue);
        return area.bottom() - (value - m_minimumValue) / span * area.height();
    }

    void drawGrid(QPainter* painter, const QRect& area)
    {
        painter->save();
        painter->setPen(QPen(QColor(62, 68, 73), 1, Qt::DashLine));
        const QFontMetrics metrics(painter->font());

        constexpr int HorizontalDivisions = 8;
        for (int i = 0; i <= HorizontalDivisions; ++i)
        {
            const int x = area.left() + qRound((double)i / HorizontalDivisions * area.width());
            painter->drawLine(x, area.top(), x, area.bottom());
            const double relativeSeconds =
                ((double)i / HorizontalDivisions * (m_endTimeMs - m_startTimeMs) +
                 m_startTimeMs - m_endTimeMs) /
                1000.0;
            const QString label = QString::number(relativeSeconds, 'f', 1);
            painter->setPen(QColor(173, 181, 189));
            painter->drawText(QRect(x - 38, area.bottom() + 8, 76, metrics.height() + 2),
                              Qt::AlignHCenter | Qt::AlignTop,
                              label);
            painter->setPen(QPen(QColor(62, 68, 73), 1, Qt::DashLine));
        }

        constexpr int VerticalDivisions = 6;
        for (int i = 0; i <= VerticalDivisions; ++i)
        {
            const int y = area.top() + qRound((double)i / VerticalDivisions * area.height());
            painter->drawLine(area.left(), y, area.right(), y);
            const double value = m_maximumValue -
                                 (double)i / VerticalDivisions *
                                     (m_maximumValue - m_minimumValue);
            const QString label = QString::number(value, 'g', 5);
            painter->setPen(QColor(173, 181, 189));
            painter->drawText(QRect(4, y - metrics.height() / 2, 54, metrics.height() + 2),
                              Qt::AlignRight | Qt::AlignVCenter,
                              label);
            painter->setPen(QPen(QColor(62, 68, 73), 1, Qt::DashLine));
        }

        painter->setPen(QPen(QColor(123, 132, 139), 1));
        painter->drawRect(area);
        painter->restore();
    }

    void drawSeries(QPainter* painter, const QRect& area)
    {
        painter->save();
        painter->setClipRect(area.adjusted(-1, -1, 1, 1));
        for (const PlotSeries& series : m_series)
        {
            QPainterPath path;
            bool pathStarted = false;
            for (const PlotPoint& point : series.points)
            {
                if (!point.valid)
                {
                    pathStarted = false;
                    continue;
                }
                const QPointF mapped(xForTime(point.timeMs, area),
                                     yForValue(point.value, area));
                if (!pathStarted)
                {
                    path.moveTo(mapped);
                    pathStarted = true;
                }
                else
                {
                    path.lineTo(mapped);
                }
            }
            painter->setPen(QPen(series.color, 1.6));
            painter->drawPath(path);
        }
        painter->restore();
    }

    void drawLegend(QPainter* painter, const QRect& area)
    {
        if (m_series.isEmpty())
        {
            painter->setPen(QColor(145, 153, 160));
            painter->drawText(area, Qt::AlignCenter, QStringLiteral("请选择需要显示的通道"));
            return;
        }

        painter->save();
        QFont font = painter->font();
        font.setPointSize(qMax(8, font.pointSize() - 1));
        painter->setFont(font);
        const QFontMetrics metrics(font);
        int x = area.left() + 8;
        int y = area.top() + 7;
        const int availableRight = area.right() - 8;
        for (const PlotSeries& series : m_series)
        {
            QString text = series.label;
            const int maximumTextWidth = qMax(70, availableRight - x - 24);
            text = metrics.elidedText(text, Qt::ElideRight, maximumTextWidth);
            const int itemWidth = 18 + metrics.horizontalAdvance(text) + 14;
            if (x + itemWidth > availableRight && x > area.left() + 8)
            {
                x = area.left() + 8;
                y += metrics.height() + 5;
            }
            painter->setPen(QPen(series.color, 3));
            painter->drawLine(x, y + metrics.height() / 2,
                              x + 12, y + metrics.height() / 2);
            painter->setPen(QColor(222, 226, 230));
            painter->drawText(QRect(x + 18, y, metrics.horizontalAdvance(text), metrics.height()),
                              Qt::AlignLeft | Qt::AlignTop,
                              text);
            x += itemWidth;
        }
        painter->restore();
    }

    void drawCursor(QPainter* painter, const QRect& area)
    {
        if (!m_cursorVisible)
        {
            return;
        }

        const double normalizedX = qBound(0.0,
                                          (m_cursorPosition.x() - area.left()) /
                                              qMax(1.0, (double)area.width()),
                                          1.0);
        const double normalizedY = qBound(0.0,
                                          (m_cursorPosition.y() - area.top()) /
                                              qMax(1.0, (double)area.height()),
                                          1.0);
        const double cursorTime = m_startTimeMs + normalizedX * (m_endTimeMs - m_startTimeMs);
        const double cursorValue = m_maximumValue - normalizedY * (m_maximumValue - m_minimumValue);

        painter->save();
        painter->setPen(QPen(QColor(205, 210, 215, 190), 1, Qt::DashLine));
        painter->drawLine(QPointF(m_cursorPosition.x(), area.top()),
                          QPointF(m_cursorPosition.x(), area.bottom()));
        painter->drawLine(QPointF(area.left(), m_cursorPosition.y()),
                          QPointF(area.right(), m_cursorPosition.y()));

        QStringList lines;
        lines.push_back(QStringLiteral("t=%1 s  y=%2")
                            .arg((cursorTime - m_endTimeMs) / 1000.0, 0, 'f', 3)
                            .arg(cursorValue, 0, 'g', 6));
        int displayedSeries = 0;
        for (const PlotSeries& series : m_series)
        {
            if (displayedSeries >= 8)
            {
                break;
            }
            const PlotPoint* nearest = nullptr;
            double nearestDistance = std::numeric_limits<double>::max();
            for (const PlotPoint& point : series.points)
            {
                if (!point.valid)
                {
                    continue;
                }
                const double distance = std::abs(point.timeMs - cursorTime);
                if (distance < nearestDistance)
                {
                    nearest = &point;
                    nearestDistance = distance;
                }
            }
            if (nearest != nullptr)
            {
                lines.push_back(QStringLiteral("%1: %2 %3")
                                    .arg(series.label)
                                    .arg(nearest->value, 0, 'g', 6)
                                    .arg(series.unit));
                ++displayedSeries;
            }
        }
        if (m_series.size() > displayedSeries)
        {
            lines.push_back(QStringLiteral("其余 %1 个通道未展开")
                                .arg(m_series.size() - displayedSeries));
        }

        const QFontMetrics metrics(painter->font());
        int textWidth = 0;
        for (const QString& line : lines)
        {
            textWidth = qMax(textWidth, metrics.horizontalAdvance(line));
        }
        const int boxWidth = qMin(textWidth + 16, area.width() - 8);
        const int boxHeight = lines.size() * metrics.height() + 12;
        int boxX = qRound(m_cursorPosition.x()) + 12;
        int boxY = qRound(m_cursorPosition.y()) + 12;
        if (boxX + boxWidth > area.right())
        {
            boxX = qRound(m_cursorPosition.x()) - boxWidth - 12;
        }
        if (boxY + boxHeight > area.bottom())
        {
            boxY = qRound(m_cursorPosition.y()) - boxHeight - 12;
        }
        boxX = qBound(area.left() + 4, boxX, area.right() - boxWidth - 4);
        boxY = qBound(area.top() + 4, boxY, area.bottom() - boxHeight - 4);
        const QRect box(boxX, boxY, boxWidth, boxHeight);
        painter->fillRect(box, QColor(25, 29, 33, 238));
        painter->setPen(QColor(94, 103, 110));
        painter->drawRect(box);
        painter->setPen(QColor(232, 235, 238));
        for (int i = 0; i < lines.size(); ++i)
        {
            painter->drawText(box.adjusted(8, 6 + i * metrics.height(), -8, 0),
                              Qt::AlignLeft | Qt::AlignTop,
                              metrics.elidedText(lines[i], Qt::ElideRight, box.width() - 16));
        }
        painter->restore();
    }

    QVector<PlotSeries> m_series;
    double m_startTimeMs = 0.0;
    double m_endTimeMs = 10000.0;
    double m_minimumValue = -1.0;
    double m_maximumValue = 1.0;
    QPointF m_cursorPosition;
    QPointF m_lastDragPosition;
    std::function<void(double, double)> m_zoomRequested;
    std::function<void(double, double)> m_panRequested;
    std::function<void()> m_followRequested;
    bool m_cursorVisible = false;
    bool m_dragging = false;
};

struct ValueBucket
{
    bool hasValue = false;
    bool hasInvalidValue = false;
    double minimumValue = 0.0;
    double maximumValue = 0.0;
    double minimumTimeMs = 0.0;
    double maximumTimeMs = 0.0;
};
}

class LogWaveformWindow::Private
{
public:
    explicit Private(LogWaveformWindow* owner)
        : q(owner)
    {
    }

    void setupUi()
    {
        q->setWindowTitle(QStringLiteral("JustFloat 实时波形"));
        q->resize(900, 520);
        q->setMinimumSize(720, 400);
        q->setWindowFlag(Qt::Window, true);

        auto* root = new QVBoxLayout(q);
        root->setContentsMargins(10, 10, 10, 10);
        root->setSpacing(8);

        auto* controls = new QGridLayout;
        controls->setHorizontalSpacing(6);
        controls->setVerticalSpacing(4);
        pauseButton = new QPushButton(QStringLiteral("暂停"), q);
        pauseButton->setFixedWidth(72);
        clearButton = new QPushButton(QStringLiteral("清空"), q);
        clearButton->setFixedWidth(72);
        followButton = new QPushButton(QStringLiteral("跟随播放"), q);
        followButton->setFixedWidth(92);
        windowSpin = new QDoubleSpinBox(q);
        windowSpin->setRange(WaveformViewport::MinimumWindowMs / 1000.0, 1000000.0);
        windowSpin->setDecimals(3);
        windowSpin->setSingleStep(0.1);
        windowSpin->setSuffix(QStringLiteral(" s"));
        windowSpin->setValue(WaveformViewport::DefaultWindowMs / 1000.0);
        windowSpin->setFixedWidth(112);
        autoYCheck = new QCheckBox(QStringLiteral("自动 Y"), q);
        autoYCheck->setChecked(true);
        minimumSpin = makeRangeSpin(-100.0);
        maximumSpin = makeRangeSpin(100.0);

        controls->addWidget(pauseButton, 0, 0);
        controls->addWidget(clearButton, 0, 1);
        controls->addWidget(followButton, 0, 2);
        controls->addWidget(new QLabel(QStringLiteral("时间窗"), q), 0, 3);
        controls->addWidget(windowSpin, 0, 4);
        controls->addWidget(autoYCheck, 0, 5);
        sourceLabel = new QLabel(QStringLiteral("CSV"), q);
        sourceLabel->setObjectName(QStringLiteral("sourceLabel"));
        controls->addWidget(sourceLabel, 0, 7, Qt::AlignRight);
        controls->addWidget(new QLabel(QStringLiteral("Y min"), q), 1, 0);
        controls->addWidget(minimumSpin, 1, 1, 1, 2);
        controls->addWidget(new QLabel(QStringLiteral("Y max"), q), 1, 3);
        controls->addWidget(maximumSpin, 1, 4, 1, 2);
        controls->setColumnStretch(7, 1);
        root->addLayout(controls);

        auto* body = new QHBoxLayout;
        body->setSpacing(8);
        plot = new WaveformPlotWidget(q);
        channelTree = new QTreeWidget(q);
        channelTree->setHeaderLabel(QStringLiteral("通道"));
        channelTree->setFixedWidth(250);
        channelTree->setRootIsDecorated(true);
        channelTree->setUniformRowHeights(true);
        channelTree->header()->setStretchLastSection(true);
        populateChannels();
        body->addWidget(plot, 1);
        body->addWidget(channelTree);
        root->addLayout(body, 1);

        plot->setNavigationCallbacks(
            [this](double wheelSteps, double anchorRatio) {
                viewport.zoom(wheelSteps, anchorRatio);
                if (paused)
                {
                    resumeFollowingAfterPause = false;
                }
                syncWindowSpin();
                updateFollowButton();
                dirty = true;
                refreshPlot();
            },
            [this](double deltaPixels, double plotWidth) {
                viewport.panPixels(deltaPixels, plotWidth);
                if (paused)
                {
                    resumeFollowingAfterPause = false;
                }
                syncWindowSpin();
                updateFollowButton();
                dirty = true;
                refreshPlot();
            },
            [this]() {
                followCurrentSource();
            });

        refreshTimer = new QTimer(q);
        refreshTimer->setInterval(RefreshIntervalMs);
        refreshTimer->start();

        QObject::connect(refreshTimer, &QTimer::timeout, q, [this]() {
            if (udpMode && liveHistory != nullptr &&
                liveHistoryRevision != liveHistory->revision())
            {
                liveHistoryRevision = liveHistory->revision();
                const bool shouldRefresh = viewport.isFollowing() && !paused;
                updateViewportBounds();
                dirty = dirty || shouldRefresh;
            }
            if (q->isVisible() && dirty && !paused)
            {
                refreshPlot();
            }
        });
        QObject::connect(pauseButton, &QPushButton::clicked, q, [this]() {
            if (!paused)
            {
                paused = true;
                resumeFollowingAfterPause = viewport.isFollowing();
                viewport.stopFollowing();
            }
            else
            {
                paused = false;
                updateViewportBounds();
                if (resumeFollowingAfterPause)
                {
                    viewport.followTarget();
                }
                resumeFollowingAfterPause = false;
                dirty = true;
                refreshPlot();
            }
            pauseButton->setText(paused ? QStringLiteral("继续") : QStringLiteral("暂停"));
            updateFollowButton();
        });
        QObject::connect(clearButton, &QPushButton::clicked, q, [this]() {
            if (udpMode)
            {
                clearLiveData();
            }
        });
        QObject::connect(followButton, &QPushButton::clicked, q, [this]() {
            followCurrentSource();
        });
        QObject::connect(windowSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), q, [this](double seconds) {
            viewport.setWindowDuration(seconds * 1000.0);
            if (paused)
            {
                resumeFollowingAfterPause = false;
            }
            syncWindowSpin();
            updateFollowButton();
            dirty = true;
            refreshPlot();
        });
        QObject::connect(autoYCheck, &QCheckBox::toggled, q, [this](bool enabled) {
            minimumSpin->setEnabled(!enabled);
            maximumSpin->setEnabled(!enabled);
            dirty = true;
            refreshPlot();
        });
        QObject::connect(minimumSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), q, [this]() {
            if (!autoYCheck->isChecked())
            {
                refreshPlot();
            }
        });
        QObject::connect(maximumSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), q, [this]() {
            if (!autoYCheck->isChecked())
            {
                refreshPlot();
            }
        });
        QObject::connect(channelTree, &QTreeWidget::itemChanged, q, [this](QTreeWidgetItem* item, int) {
            if (item->data(0, Qt::UserRole).isValid())
            {
                dirty = true;
                refreshPlot();
            }
        });

        minimumSpin->setEnabled(false);
        maximumSpin->setEnabled(false);
        applyStyle();
        updateModeControls();
        viewport.reset();
        syncWindowSpin();
        updateFollowButton();
        refreshPlot();
    }

    void setUdpMode(bool enabled)
    {
        if (udpMode == enabled)
        {
            updateViewportBounds();
            dirty = true;
            if (!paused)
            {
                refreshPlot();
            }
            return;
        }
        udpMode = enabled;
        paused = false;
        resumeFollowingAfterPause = false;
        pauseButton->setText(QStringLiteral("暂停"));
        viewport.reset();
        updateViewportBounds();
        dirty = true;
        updateModeControls();
        syncWindowSpin();
        updateFollowButton();
        if (!paused)
        {
            refreshPlot();
        }
    }

    void setLiveHistory(WaveformHistoryStore* history)
    {
        if (liveHistory == history)
        {
            const bool resetForNewSession =
                liveHistory != nullptr &&
                liveHistoryRevision != liveHistory->revision() &&
                liveHistory->sampleCount() == 0;
            liveHistoryRevision = liveHistory != nullptr ? liveHistory->revision() : 0;
            if (resetForNewSession)
            {
                viewport.reset();
            }
            updateViewportBounds();
            syncWindowSpin();
            updateFollowButton();
            dirty = true;
            if (udpMode && !paused)
            {
                refreshPlot();
            }
            return;
        }
        liveHistory = history;
        liveHistoryRevision = liveHistory != nullptr ? liveHistory->revision() : 0;
        viewport.reset();
        updateViewportBounds();
        dirty = true;
        syncWindowSpin();
        updateFollowButton();
        if (udpMode && !paused)
        {
            refreshPlot();
        }
    }

    void configureLiveSource(const QString& sourceName, const QVector<int>& channels)
    {
        liveSourceName = sourceName.trimmed().isEmpty() ? QStringLiteral("UDP") : sourceName;
        if (!channels.isEmpty())
        {
            for (int index = 0; index < channelItems.size(); ++index)
            {
                QTreeWidgetItem* item = channelItems[index];
                if (item == nullptr)
                {
                    continue;
                }
                const bool visible = channels.contains(index);
                item->setHidden(!visible);
                item->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
            }
            for (int groupIndex = 0; groupIndex < channelTree->topLevelItemCount(); ++groupIndex)
            {
                QTreeWidgetItem* group = channelTree->topLevelItem(groupIndex);
                bool visible = false;
                for (int childIndex = 0; childIndex < group->childCount(); ++childIndex)
                {
                    visible = visible || !group->child(childIndex)->isHidden();
                }
                group->setHidden(!visible);
                group->setExpanded(visible);
            }
        }
        updateModeControls();
        dirty = true;
        if (!paused)
        {
            refreshPlot();
        }
    }

    void clearLiveData()
    {
        paused = false;
        resumeFollowingAfterPause = false;
        pauseButton->setText(QStringLiteral("暂停"));
        if (liveHistory != nullptr)
        {
            QString error;
            if (!liveHistory->clear(&error))
            {
                sourceLabel->setToolTip(error);
                return;
            }
            liveHistoryRevision = liveHistory->revision();
        }
        viewport.reset();
        updateViewportBounds();
        syncWindowSpin();
        updateFollowButton();
        dirty = true;
        refreshPlot();
    }

    void setCsvLog(const JustFloatLog* log)
    {
        paused = false;
        resumeFollowingAfterPause = false;
        pauseButton->setText(QStringLiteral("暂停"));
        csvLog = log;
        csvTimeline.clear();
        if (csvLog != nullptr && csvLog->rowCount() > 0)
        {
            csvTimeline.resize(csvLog->rowCount());
            double lastSource = std::numeric_limits<double>::quiet_NaN();
            for (int i = 0; i < csvLog->rowCount(); ++i)
            {
                const double source = sourceTimeMs(csvLog->rowAt(i));
                if (i == 0)
                {
                    csvTimeline[i] = std::isfinite(source) ? source : 0.0;
                }
                else
                {
                    double delta = std::numeric_limits<double>::quiet_NaN();
                    if (std::isfinite(source) && std::isfinite(lastSource))
                    {
                        delta = source - lastSource;
                    }
                    if (!std::isfinite(delta) || delta <= 0.0)
                    {
                        delta = 20.0;
                    }
                    csvTimeline[i] = csvTimeline[i - 1] + delta;
                }
                lastSource = source;
            }
            csvRow = qBound(0, csvRow, csvLog->rowCount() - 1);
        }
        else
        {
            csvRow = -1;
        }
        viewport.reset();
        updateViewportBounds();
        syncWindowSpin();
        updateFollowButton();
        dirty = true;
        if (!udpMode && !paused)
        {
            refreshPlot();
        }
    }

    void setCsvRow(int row)
    {
        if (csvLog == nullptr || csvLog->rowCount() <= 0)
        {
            csvRow = -1;
        }
        else
        {
            csvRow = qBound(0, row, csvLog->rowCount() - 1);
        }
        const bool shouldRefresh = viewport.isFollowing() && !paused;
        updateViewportBounds();
        updateFollowButton();
        dirty = dirty || shouldRefresh;
    }

private:
    void updateViewportBounds()
    {
        if (udpMode)
        {
            if (liveHistory != nullptr && liveHistory->sampleCount() > 0)
            {
                viewport.setBounds(liveHistory->firstTimeMs(), liveHistory->lastTimeMs());
                viewport.setFollowTarget(liveHistory->lastTimeMs());
            }
            else
            {
                viewport.setBounds(0.0, 0.0);
                viewport.setFollowTarget(0.0);
            }
            return;
        }

        if (!csvTimeline.isEmpty())
        {
            viewport.setBounds(csvTimeline.front(), csvTimeline.back());
            const int targetRow = qBound(0,
                                         csvRow,
                                         static_cast<int>(csvTimeline.size()) - 1);
            viewport.setFollowTarget(csvTimeline[targetRow]);
        }
        else
        {
            viewport.setBounds(0.0, 0.0);
            viewport.setFollowTarget(0.0);
        }
    }

    void syncWindowSpin()
    {
        const QSignalBlocker blocker(windowSpin);
        windowSpin->setValue(viewport.windowDurationMs() / 1000.0);
    }

    void updateFollowButton()
    {
        followButton->setText(udpMode ? QStringLiteral("回到实时")
                                      : QStringLiteral("跟随播放"));
        followButton->setEnabled(!viewport.isFollowing() || paused);
    }

    void followCurrentSource()
    {
        paused = false;
        resumeFollowingAfterPause = false;
        pauseButton->setText(QStringLiteral("暂停"));
        updateViewportBounds();
        viewport.followTarget();
        syncWindowSpin();
        updateFollowButton();
        dirty = true;
        refreshPlot();
    }

    QDoubleSpinBox* makeRangeSpin(double value)
    {
        auto* spin = new QDoubleSpinBox(q);
        spin->setRange(-1000000000.0, 1000000000.0);
        spin->setDecimals(3);
        spin->setValue(value);
        spin->setFixedWidth(104);
        return spin;
    }

    void populateChannels()
    {
        channelItems.fill(nullptr, JustFloatLog::ChannelCount);
        QHash<QString, QTreeWidgetItem*> groups;
        const auto& descriptors = JustFloatLog::channelDescriptors();
        for (const JustFloatChannelDescriptor& descriptor : descriptors)
        {
            QTreeWidgetItem* group = groups.value(descriptor.group, nullptr);
            if (group == nullptr)
            {
                group = new QTreeWidgetItem(channelTree, QStringList(descriptor.group));
                group->setFlags(group->flags() & ~Qt::ItemIsSelectable);
                group->setExpanded(descriptor.index >= 34);
                groups.insert(descriptor.group, group);
            }

            auto* item = new QTreeWidgetItem(group, QStringList(channelLabel(descriptor)));
            item->setData(0, Qt::UserRole, descriptor.index);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(0,
                                descriptor.index == 38 || descriptor.index == 39
                                    ? Qt::Checked
                                    : Qt::Unchecked);
            QPixmap swatch(12, 12);
            swatch.fill(channelColor(descriptor.index));
            item->setIcon(0, QIcon(swatch));
            channelItems[descriptor.index] = item;
        }
    }

    void applyStyle()
    {
        q->setStyleSheet(QStringLiteral(
            "QWidget { background:#111315; color:#e5e7eb; }"
            "QPushButton, QComboBox, QDoubleSpinBox {"
            " background:#202428; border:1px solid #3a4147; border-radius:4px;"
            " min-height:26px; padding:2px 8px; }"
            "QPushButton:hover { border-color:#f6d44a; }"
            "QPushButton:disabled { color:#737b82; background:#181b1e; }"
            "QCheckBox { spacing:6px; }"
            "QTreeWidget { background:#171a1d; border:1px solid #343a40; }"
            "QTreeWidget::item { min-height:23px; }"
            "QTreeWidget::item:selected { background:#3c4650; }"
            "QHeaderView::section { background:#202428; border:0; border-bottom:1px solid #3a4147;"
            " padding:5px; font-weight:600; }"
            "QLabel#sourceLabel { color:#f6d44a; font-weight:600; padding:3px 8px;"
            " border:1px solid #5c552c; border-radius:4px; background:#242216; }"));
    }

    void updateModeControls()
    {
        sourceLabel->setText(udpMode
                                 ? QStringLiteral("%1 实时").arg(liveSourceName)
                                 : QStringLiteral("CSV 回放"));
        clearButton->setEnabled(udpMode);
        q->setWindowTitle(udpMode
                              ? (liveSourceName == QStringLiteral("UDP")
                                     ? QStringLiteral("JustFloat 实时波形 - UDP")
                                     : QStringLiteral("姿态实时波形 - %1").arg(liveSourceName))
                              : QStringLiteral("JustFloat 实时波形 - CSV"));
        updateFollowButton();
    }

    QVector<int> selectedChannels() const
    {
        QVector<int> result;
        result.reserve(JustFloatLog::ChannelCount);
        for (int i = 0; i < channelItems.size(); ++i)
        {
            if (channelItems[i] != nullptr && channelItems[i]->checkState(0) == Qt::Checked)
            {
                result.push_back(i);
            }
        }
        return result;
    }

    QVector<PlotSeries> buildCsvSeries(double startTimeMs,
                                       double endTimeMs,
                                       double* dataMinimum,
                                       double* dataMaximum) const
    {
        const QVector<int> channels = selectedChannels();
        QVector<PlotSeries> series;
        series.reserve(channels.size());
        const int width = qMax(1, plot->plotPixelWidth());
        const double span = qMax(1.0, endTimeMs - startTimeMs);
        *dataMinimum = std::numeric_limits<double>::infinity();
        *dataMaximum = -std::numeric_limits<double>::infinity();

        const auto& descriptors = JustFloatLog::channelDescriptors();
        const int count = csvLog != nullptr ? csvLog->rowCount() : 0;
        const int lastIndex = count - 1;
        int firstIndex = 0;
        if (!csvTimeline.isEmpty())
        {
            firstIndex = static_cast<int>(std::lower_bound(csvTimeline.cbegin(),
                                                            csvTimeline.cend(),
                                                            startTimeMs) -
                                          csvTimeline.cbegin());
        }
        const int endIndex = csvTimeline.isEmpty()
                                 ? -1
                                 : static_cast<int>(std::upper_bound(csvTimeline.cbegin(),
                                                                     csvTimeline.cend(),
                                                                     endTimeMs) -
                                                        csvTimeline.cbegin()) -
                                       1;
        for (int channel : channels)
        {
            QVector<ValueBucket> buckets(width);
            for (int i = firstIndex; i <= qMin(lastIndex, endIndex); ++i)
            {
                if (csvLog == nullptr || i < 0 || i >= csvLog->rowCount())
                {
                    continue;
                }
                const JustFloatLogRow& row = csvLog->rowAt(i);
                const double timeMs = csvTimeline[i];
                double value = 0.0;
                const int bucketIndex = qBound(0,
                                               static_cast<int>((timeMs - startTimeMs) / span * width),
                                               width - 1);
                ValueBucket& bucket = buckets[bucketIndex];
                if (!JustFloatLog::channelValue(row, channel, &value) ||
                    !isUsableChannelValue(row, channel, value))
                {
                    bucket.hasInvalidValue = true;
                    continue;
                }
                if (!bucket.hasValue)
                {
                    bucket.hasValue = true;
                    bucket.minimumValue = value;
                    bucket.maximumValue = value;
                    bucket.minimumTimeMs = timeMs;
                    bucket.maximumTimeMs = timeMs;
                }
                else
                {
                    if (value < bucket.minimumValue)
                    {
                        bucket.minimumValue = value;
                        bucket.minimumTimeMs = timeMs;
                    }
                    if (value > bucket.maximumValue)
                    {
                        bucket.maximumValue = value;
                        bucket.maximumTimeMs = timeMs;
                    }
                }
                *dataMinimum = qMin(*dataMinimum, value);
                *dataMaximum = qMax(*dataMaximum, value);
            }

            PlotSeries item;
            item.channelIndex = channel;
            item.label = channelLabel(descriptors[channel]);
            item.unit = descriptors[channel].unit;
            item.color = channelColor(channel);
            item.points.reserve(width * 2);
            bool previousBucketHadValue = false;
            for (const ValueBucket& bucket : buckets)
            {
                if (!bucket.hasValue)
                {
                    if (previousBucketHadValue && bucket.hasInvalidValue)
                    {
                        item.points.push_back({0.0, 0.0, false});
                    }
                    if (bucket.hasInvalidValue)
                    {
                        previousBucketHadValue = false;
                    }
                    continue;
                }

                if (bucket.hasInvalidValue && previousBucketHadValue)
                {
                    item.points.push_back({0.0, 0.0, false});
                    previousBucketHadValue = false;
                }

                if (bucket.minimumTimeMs <= bucket.maximumTimeMs)
                {
                    item.points.push_back({bucket.minimumTimeMs, bucket.minimumValue, true});
                    if (bucket.maximumTimeMs != bucket.minimumTimeMs ||
                        bucket.maximumValue != bucket.minimumValue)
                    {
                        item.points.push_back({bucket.maximumTimeMs, bucket.maximumValue, true});
                    }
                }
                else
                {
                    item.points.push_back({bucket.maximumTimeMs, bucket.maximumValue, true});
                    item.points.push_back({bucket.minimumTimeMs, bucket.minimumValue, true});
                }
                if (bucket.hasInvalidValue)
                {
                    item.points.push_back({0.0, 0.0, false});
                    previousBucketHadValue = false;
                }
                else
                {
                    previousBucketHadValue = true;
                }
            }
            series.push_back(std::move(item));
        }
        return series;
    }

    QVector<PlotSeries> buildLiveSeries(double startTimeMs,
                                        double endTimeMs,
                                        double* dataMinimum,
                                        double* dataMaximum)
    {
        *dataMinimum = std::numeric_limits<double>::infinity();
        *dataMaximum = -std::numeric_limits<double>::infinity();
        const QVector<int> channels = selectedChannels();
        QVector<PlotSeries> result;
        if (liveHistory == nullptr)
        {
            return result;
        }

        QString error;
        const QVector<WaveformHistorySeries> historySeries =
            liveHistory->query(channels,
                               startTimeMs,
                               endTimeMs,
                               qMax(1, plot->plotPixelWidth()),
                               &error);
        if (!error.isEmpty())
        {
            sourceLabel->setToolTip(error);
        }

        const auto& descriptors = JustFloatLog::channelDescriptors();
        result.reserve(historySeries.size());
        for (const WaveformHistorySeries& source : historySeries)
        {
            PlotSeries item;
            item.channelIndex = source.channelIndex;
            item.label = channelLabel(descriptors[source.channelIndex]);
            item.unit = descriptors[source.channelIndex].unit;
            item.color = channelColor(source.channelIndex);
            item.points.reserve(source.points.size());
            for (const WaveformHistoryPoint& point : source.points)
            {
                item.points.push_back({point.timeMs, point.value, point.valid});
                if (point.valid)
                {
                    *dataMinimum = qMin(*dataMinimum, point.value);
                    *dataMaximum = qMax(*dataMaximum, point.value);
                }
            }
            result.push_back(std::move(item));
        }
        return result;
    }

    void refreshPlot()
    {
        dirty = false;
        updateViewportBounds();
        syncWindowSpin();
        updateFollowButton();
        const double startTimeMs = viewport.startTimeMs();
        const double endTimeMs = viewport.endTimeMs();

        double dataMinimum = std::numeric_limits<double>::infinity();
        double dataMaximum = -std::numeric_limits<double>::infinity();
        QVector<PlotSeries> series = udpMode
                                         ? buildLiveSeries(startTimeMs,
                                                           endTimeMs,
                                                           &dataMinimum,
                                                           &dataMaximum)
                                         : buildCsvSeries(startTimeMs,
                                                          endTimeMs,
                                                          &dataMinimum,
                                                          &dataMaximum);

        double minimumValue = minimumSpin->value();
        double maximumValue = maximumSpin->value();
        if (autoYCheck->isChecked())
        {
            if (std::isfinite(dataMinimum) && std::isfinite(dataMaximum))
            {
                double margin = (dataMaximum - dataMinimum) * 0.05;
                if (margin < 1e-6)
                {
                    margin = qMax(1.0, std::abs(dataMaximum) * 0.05);
                }
                minimumValue = dataMinimum - margin;
                maximumValue = dataMaximum + margin;
            }
            else
            {
                minimumValue = -1.0;
                maximumValue = 1.0;
            }
        }
        else if (!(maximumValue > minimumValue))
        {
            maximumValue = minimumValue + 1.0;
        }

        plot->setPlotData(std::move(series),
                          startTimeMs,
                          endTimeMs,
                          minimumValue,
                          maximumValue);
    }

public:
    LogWaveformWindow* q = nullptr;
    WaveformPlotWidget* plot = nullptr;
    QPushButton* pauseButton = nullptr;
    QPushButton* clearButton = nullptr;
    QPushButton* followButton = nullptr;
    QDoubleSpinBox* windowSpin = nullptr;
    QCheckBox* autoYCheck = nullptr;
    QDoubleSpinBox* minimumSpin = nullptr;
    QDoubleSpinBox* maximumSpin = nullptr;
    QLabel* sourceLabel = nullptr;
    QTreeWidget* channelTree = nullptr;
    QTimer* refreshTimer = nullptr;
    QVector<QTreeWidgetItem*> channelItems;
    WaveformHistoryStore* liveHistory = nullptr;
    WaveformViewport viewport;
    quint64 liveHistoryRevision = 0;
    const JustFloatLog* csvLog = nullptr;
    QVector<double> csvTimeline;
    QString liveSourceName = QStringLiteral("UDP");
    int csvRow = -1;
    bool udpMode = false;
    bool paused = false;
    bool resumeFollowingAfterPause = false;
    bool dirty = true;
};

LogWaveformWindow::LogWaveformWindow(QWidget* parent)
    : QWidget(parent),
      d(std::make_unique<Private>(this))
{
    d->setupUi();
}

LogWaveformWindow::~LogWaveformWindow() = default;

void LogWaveformWindow::setUdpMode(bool enabled)
{
    d->setUdpMode(enabled);
}

void LogWaveformWindow::setLiveHistory(WaveformHistoryStore* history)
{
    d->setLiveHistory(history);
}

void LogWaveformWindow::configureLiveSource(const QString& sourceName,
                                            const QVector<int>& channels)
{
    d->configureLiveSource(sourceName, channels);
}

void LogWaveformWindow::clearLiveData()
{
    d->clearLiveData();
}

void LogWaveformWindow::setCsvLog(const JustFloatLog* log)
{
    d->setCsvLog(log);
}

void LogWaveformWindow::setCsvRow(int row)
{
    d->setCsvRow(row);
}

void LogWaveformWindow::closeEvent(QCloseEvent* event)
{
    event->ignore();
    hide();
}
